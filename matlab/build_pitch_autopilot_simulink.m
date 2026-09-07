function mdl = build_pitch_autopilot_simulink(cfg, des, mdlname, opts)
%BUILD_PITCH_AUTOPILOT_SIMULINK  Build the pitch-autopilot Simulink model.
%
%   mdl = build_pitch_autopilot_simulink(cfg, des)
%   mdl = build_pitch_autopilot_simulink(cfg, des, 'my_model')
%   mdl = build_pitch_autopilot_simulink(cfg, des, 'my_model', opts)
%
%   Creates (and saves) a model containing one autopilot branch per
%   controller, all driven by the SAME attitude command, gust and mis-trim
%   sources so the three designs can be compared on one set of scopes.
%
%   Each branch is the complete loop:
%
%     command --> controller --> asymmetric travel saturation
%             --> rate-limited first-order servo --> airframe --> theta
%                                    |                              |
%                                    +----- observer (theta only) <-+
%
%   The elevator model is built from primitives so that it matches the
%   MATLAB simulation exactly:
%       delta_dot = sat_rate( (sat_travel(u_cmd) - delta)/tau ),
%       delta itself limited to [elev_dn, elev_up]
%   and the integral terms use back-calculation anti-windup
%       xi_dot = e + (u_sat - u_cmd)/tau_aw.
%
%   INPUTS
%     cfg  - the configuration struct built by aircraft_pitch_autopilot.m
%     des  - struct with fields A, B, C, Bd, L, PID, LQR, LQI (the designed
%            controllers, as produced by that script)
%     opts - optional struct:
%              .controllers  cell array, subset of {'PID','LQR','LQI'}
%              .stop_time    simulation stop time (s)
%              .gust_time    when the gust hits (s)
%              .shape        true to insert the command-shaping filter
%              .open         true to open the model when it is built
%
%   OUTPUT
%     mdl  - the model name
%
%   The gust is injected as a pitch-rate disturbance pulse whose AREA is
%   the requested attitude upset, so theta is displaced by gust_theta_deg
%   and stays displaced until the autopilot corrects it (a rectangular
%   pulse here, the raised cosine of the MATLAB script has the same area).

% ---------------- arguments ----------------------------------------------
if nargin < 3 || isempty(mdlname), mdlname = 'pitch_autopilot'; end
if nargin < 4 || isempty(opts),    opts = struct(); end
if ~isfield(opts, 'controllers'), opts.controllers = {'PID','LQR','LQI'}; end
if ~isfield(opts, 'stop_time'),   opts.stop_time   = cfg.T_long;          end
if ~isfield(opts, 'gust_time'),   opts.gust_time   = 0.5*cfg.T_long;      end
if ~isfield(opts, 'shape'),       opts.shape       = false;               end
if ~isfield(opts, 'open'),        opts.open        = false;               end

D2R = pi/180;
A   = des.A;  B = des.B;  C = des.C;  Bd = des.Bd;  L = des.L;
Tg  = max(cfg.gust_rise_s, 1e-3);

% ---------------- fresh model -------------------------------------------
if bdIsLoaded(mdlname), close_system(mdlname, 0); end
for ext = {'.slx', '.mdl'}                 % keep any previous model as a backup
    old = [mdlname ext{1}];
    if exist(old, 'file')
        movefile(old, [old '.bak']);
        fprintf('  existing %s moved to %s.bak\n', old, old);
    end
end
new_system(mdlname);
set_param(mdlname, 'Solver', 'ode45', 'StopTime', num2str(opts.stop_time), ...
                   'MaxStep', '0.01', 'RelTol', '1e-6');

% ---------------- shared sources ----------------------------------------
blk(mdlname, 'simulink/Sources/Step', 'Attitude_command', [30 30 70 70], ...
    'Time', '0', 'Before', '0', 'After', num2str(cfg.target_theta_deg*D2R, 10));

cmdSrc = 'Attitude_command/1';
if opts.shape && cfg.shape_tau > 0
    blk(mdlname, 'simulink/Continuous/Transfer Fcn', 'Command_shaping', [100 30 160 70], ...
        'Numerator', '[1]', 'Denominator', mat2str([cfg.shape_tau 1], 10));
    wire(mdlname, cmdSrc, 'Command_shaping/1');
    cmdSrc = 'Command_shaping/1';
end

% gust: rectangular pitch-rate pulse, area = the requested attitude upset
blk(mdlname, 'simulink/Sources/Pulse Generator', 'Gust', [30 110 70 150], ...
    'PulseType', 'Time based', ...
    'Amplitude', num2str(cfg.gust_theta_deg*D2R/Tg, 10), ...
    'Period',    num2str(10*opts.stop_time, 10), ...
    'PulseWidth', num2str(100*Tg/(10*opts.stop_time), 10), ...
    'PhaseDelay', num2str(opts.gust_time, 10));

% standing mis-trim: leave the amplitude at 0 and type a value in to test it
blk(mdlname, 'simulink/Sources/Step', 'Mistrim_elevator', [30 190 70 230], ...
    'Time', num2str(cfg.bias_time_s, 10), 'Before', '0', 'After', '0');

nb = numel(opts.controllers);
blk(mdlname, 'simulink/Sources/Constant', 'Attitude_target_deg', [1180 30 1240 70], ...
    'Value', num2str(cfg.theta_target_abs_deg, 10));
blk(mdlname, 'simulink/Sources/Constant', 'Elevator_limit_up', [1180 110 1240 150], ...
    'Value', num2str(cfg.elev_up_deg, 10));
blk(mdlname, 'simulink/Sources/Constant', 'Elevator_limit_dn', [1180 190 1240 230], ...
    'Value', num2str(cfg.elev_dn_deg, 10));
blk(mdlname, 'simulink/Signal Routing/Mux', 'Mux_attitude', [1300 40 1305 40 + 30*(nb+1)], ...
    'Inputs', num2str(nb+1));
blk(mdlname, 'simulink/Signal Routing/Mux', 'Mux_elevator', [1300 300 1305 300 + 30*(nb+2)], ...
    'Inputs', num2str(nb+2));
blk(mdlname, 'simulink/Sinks/Scope', 'Attitude_deg', [1360 60 1400 100]);
blk(mdlname, 'simulink/Sinks/Scope', 'Elevator_deg',  [1360 320 1400 360]);
wire(mdlname, 'Mux_attitude/1', 'Attitude_deg/1');
wire(mdlname, 'Mux_elevator/1', 'Elevator_deg/1');

% ---------------- one branch per controller ------------------------------
for k = 1:nb
    nm = opts.controllers{k};
    y0 = 300 + (k-1)*320;
    switch upper(nm)
        case 'PID', ctrl = des.PID;
        case 'LQR', ctrl = des.LQR;
        case 'LQI', ctrl = des.LQI;
        otherwise,  error('unknown controller %s', nm);
    end
    addBranch(mdlname, nm, ctrl, cfg, A, B, C, Bd, L, y0, cmdSrc, D2R);
    wire(mdlname, ['Attitude_abs_' nm '/1'], sprintf('Mux_attitude/%d', k));
    wire(mdlname, ['Elevator_cmd_' nm '/1'], sprintf('Mux_elevator/%d', k));
end
wire(mdlname, 'Attitude_target_deg/1', sprintf('Mux_attitude/%d', nb+1));
wire(mdlname, 'Elevator_limit_up/1',   sprintf('Mux_elevator/%d', nb+1));
wire(mdlname, 'Elevator_limit_dn/1',   sprintf('Mux_elevator/%d', nb+2));

% ---------------- annotate, tidy, save -----------------------------------
try
    note = sprintf(['Pitch autopilot: %+.1f deg command from %+.1f deg trim, ' ...
                    '%+.1f deg gust at t = %.1f s.\n' ...
                    'Elevator travel %+.0f / %+.0f deg, rate %.0f deg/s, servo %.2f s.\n' ...
                    'Scopes and To Workspace signals are in DEGREES.'], ...
                    cfg.target_theta_deg, cfg.theta_trim_deg, cfg.gust_theta_deg, ...
                    opts.gust_time, cfg.elev_up_deg, cfg.elev_dn_deg, ...
                    cfg.elev_rate_deg, cfg.tau_act);
    an = Simulink.Annotation([mdlname '/autopilot_notes']);
    an.Text = note;  an.Position = [30 260];
catch
    % annotations are cosmetic only
end
try
    Simulink.BlockDiagram.arrangeSystem(mdlname);
catch
end
save_system(mdlname);
if opts.open, open_system(mdlname); end
mdl = mdlname;
end

% =========================================================================
function addBranch(mdl, nm, ctrl, cfg, A, B, C, Bd, L, y0, cmdSrc, D2R)
% One complete autopilot channel: controller -> limits -> servo -> airframe
% -> observer, plus the degree-valued outputs used by the scopes.
h  = 40;
x  = @(col) 100 + 110*col;
pos = @(col, row, w) [x(col), y0 + 70*row, x(col) + w, y0 + 70*row + h];

useObs = cfg.use_observer && ~strcmp(ctrl.type, 'PID') && ~strcmp(ctrl.type, 'P');

% ---- controller ---------------------------------------------------------
switch ctrl.type
    case {'P','PID'}
        blk(mdl, 'simulink/Math Operations/Sum', ['Err_' nm], pos(0,0,30), 'Inputs', '+-');
        wire(mdl, cmdSrc, ['Err_' nm '/1']);
        blk(mdl, 'simulink/Math Operations/Gain', ['Kp_' nm], pos(1,0,40), ...
            'Gain', num2str(ctrl.Kp, 10));
        wire(mdl, ['Err_' nm '/1'], ['Kp_' nm '/1']);
        % integral path with back-calculation anti-windup
        blk(mdl, 'simulink/Math Operations/Sum', ['Sum_int_' nm], pos(1,1,30), 'Inputs', '++');
        wire(mdl, ['Err_' nm '/1'], ['Sum_int_' nm '/1']);
        blk(mdl, 'simulink/Continuous/Integrator', ['Int_' nm], pos(2,1,40), ...
            'InitialCondition', '0');
        wire(mdl, ['Sum_int_' nm '/1'], ['Int_' nm '/1']);
        blk(mdl, 'simulink/Math Operations/Gain', ['Ki_' nm], pos(3,1,40), ...
            'Gain', num2str(ctrl.Ki, 10));
        wire(mdl, ['Int_' nm '/1'], ['Ki_' nm '/1']);
        % filtered derivative ON THE MEASUREMENT (no derivative kick)
        blk(mdl, 'simulink/Continuous/Transfer Fcn', ['Dfilt_' nm], pos(1,2,60), ...
            'Numerator', mat2str([ctrl.Kd*ctrl.N 0], 10), ...
            'Denominator', mat2str([1 ctrl.N], 10));
        blk(mdl, 'simulink/Math Operations/Sum', ['U_' nm], pos(4,0,30), 'Inputs', '++-');
        wire(mdl, ['Kp_' nm '/1'], ['U_' nm '/1']);
        wire(mdl, ['Ki_' nm '/1'], ['U_' nm '/2']);
        wire(mdl, ['Dfilt_' nm '/1'], ['U_' nm '/3']);
        uCmd = ['U_' nm '/1'];
    case 'SF'
        blk(mdl, 'simulink/Math Operations/Gain', ['Ref_vec_' nm], pos(0,0,50), ...
            'Gain', mat2str([0; 0; 1]), 'Multiplication', 'Matrix(K*u)');
        wire(mdl, cmdSrc, ['Ref_vec_' nm '/1']);
        blk(mdl, 'simulink/Math Operations/Sum', ['Xerr_' nm], pos(1,0,30), 'Inputs', '+-');
        wire(mdl, ['Ref_vec_' nm '/1'], ['Xerr_' nm '/2']);
        blk(mdl, 'simulink/Math Operations/Gain', ['Kmat_' nm], pos(2,0,50), ...
            'Gain', mat2str(-ctrl.K, 10), 'Multiplication', 'Matrix(K*u)');
        wire(mdl, ['Xerr_' nm '/1'], ['Kmat_' nm '/1']);
        uCmd = ['Kmat_' nm '/1'];
    case 'LQI'
        blk(mdl, 'simulink/Math Operations/Sum', ['Err_' nm], pos(0,0,30), 'Inputs', '+-');
        wire(mdl, cmdSrc, ['Err_' nm '/1']);
        blk(mdl, 'simulink/Math Operations/Sum', ['Sum_int_' nm], pos(1,1,30), 'Inputs', '++');
        wire(mdl, ['Err_' nm '/1'], ['Sum_int_' nm '/1']);
        blk(mdl, 'simulink/Continuous/Integrator', ['Int_' nm], pos(2,1,40), ...
            'InitialCondition', '0');
        wire(mdl, ['Sum_int_' nm '/1'], ['Int_' nm '/1']);
        blk(mdl, 'simulink/Math Operations/Gain', ['Kz_' nm], pos(3,1,40), ...
            'Gain', num2str(-ctrl.Kz, 10));
        wire(mdl, ['Int_' nm '/1'], ['Kz_' nm '/1']);
        blk(mdl, 'simulink/Math Operations/Gain', ['Kx_' nm], pos(2,0,50), ...
            'Gain', mat2str(-ctrl.Kx, 10), 'Multiplication', 'Matrix(K*u)');
        blk(mdl, 'simulink/Math Operations/Sum', ['U_' nm], pos(4,0,30), 'Inputs', '++');
        wire(mdl, ['Kx_' nm '/1'], ['U_' nm '/1']);
        wire(mdl, ['Kz_' nm '/1'], ['U_' nm '/2']);
        uCmd = ['U_' nm '/1'];
end

% ---- elevator: travel saturation, then a rate-limited first-order servo --
blk(mdl, 'simulink/Discontinuities/Saturation', ['Travel_sat_' nm], pos(5,0,40), ...
    'UpperLimit', num2str(cfg.u_max, 10), 'LowerLimit', num2str(cfg.u_min, 10));
wire(mdl, uCmd, ['Travel_sat_' nm '/1']);

if ctrl.has_integrator            % back-calculation anti-windup
    blk(mdl, 'simulink/Math Operations/Sum', ['AW_err_' nm], pos(5,3,30), 'Inputs', '+-');
    wire(mdl, ['Travel_sat_' nm '/1'], ['AW_err_' nm '/1']);
    wire(mdl, uCmd, ['AW_err_' nm '/2']);
    blk(mdl, 'simulink/Math Operations/Gain', ['AW_gain_' nm], pos(4,3,40), ...
        'Gain', num2str(1/cfg.tau_aw, 10));
    wire(mdl, ['AW_err_' nm '/1'], ['AW_gain_' nm '/1']);
    wire(mdl, ['AW_gain_' nm '/1'], ['Sum_int_' nm '/2']);
end

blk(mdl, 'simulink/Math Operations/Sum', ['Servo_err_' nm], pos(6,0,30), 'Inputs', '+-');
wire(mdl, ['Travel_sat_' nm '/1'], ['Servo_err_' nm '/1']);
blk(mdl, 'simulink/Math Operations/Gain', ['Servo_gain_' nm], pos(7,0,40), ...
    'Gain', num2str(1/cfg.tau_act, 10));
wire(mdl, ['Servo_err_' nm '/1'], ['Servo_gain_' nm '/1']);
blk(mdl, 'simulink/Discontinuities/Saturation', ['Rate_lim_' nm], pos(8,0,40), ...
    'UpperLimit', num2str(cfg.u_rate, 10), 'LowerLimit', num2str(-cfg.u_rate, 10));
wire(mdl, ['Servo_gain_' nm '/1'], ['Rate_lim_' nm '/1']);
blk(mdl, 'simulink/Continuous/Integrator', ['Elevator_' nm], pos(9,0,40), ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', num2str(cfg.u_max, 10), ...
    'LowerSaturationLimit', num2str(cfg.u_min, 10));
wire(mdl, ['Rate_lim_' nm '/1'], ['Elevator_' nm '/1']);
wire(mdl, ['Elevator_' nm '/1'], ['Servo_err_' nm '/2']);

% ---- airframe -----------------------------------------------------------
blk(mdl, 'simulink/Math Operations/Sum', ['Elev_plus_bias_' nm], pos(10,0,30), 'Inputs', '++');
wire(mdl, ['Elevator_' nm '/1'], ['Elev_plus_bias_' nm '/1']);
wire(mdl, 'Mistrim_elevator/1',  ['Elev_plus_bias_' nm '/2']);
blk(mdl, 'simulink/Signal Routing/Mux', ['Plant_in_' nm], ...
    [x(11), y0 + 5, x(11) + 5, y0 + 65], 'Inputs', '2');
wire(mdl, ['Elev_plus_bias_' nm '/1'], ['Plant_in_' nm '/1']);
wire(mdl, 'Gust/1', ['Plant_in_' nm '/2']);
blk(mdl, 'simulink/Continuous/State-Space', ['Airframe_' nm], pos(12,0,70), ...
    'A', mat2str(A, 10), 'B', mat2str([B Bd], 10), ...
    'C', mat2str(eye(3)), 'D', mat2str(zeros(3,2)), 'X0', '[0;0;0]');
wire(mdl, ['Plant_in_' nm '/1'], ['Airframe_' nm '/1']);
blk(mdl, 'simulink/Math Operations/Gain', ['Theta_' nm], pos(13,0,40), ...
    'Gain', mat2str([0 0 1]), 'Multiplication', 'Matrix(K*u)');
wire(mdl, ['Airframe_' nm '/1'], ['Theta_' nm '/1']);

% ---- feedback -----------------------------------------------------------
switch ctrl.type
    case {'P','PID'}
        wire(mdl, ['Theta_' nm '/1'], ['Err_' nm '/2']);
        wire(mdl, ['Theta_' nm '/1'], ['Dfilt_' nm '/1']);
    case 'SF'
        if useObs
            addObserver(mdl, nm, A, B, C, L, x, y0, h);
            wire(mdl, ['Observer_' nm '/1'], ['Xerr_' nm '/1']);
        else
            wire(mdl, ['Airframe_' nm '/1'], ['Xerr_' nm '/1']);
        end
    case 'LQI'
        wire(mdl, ['Theta_' nm '/1'], ['Err_' nm '/2']);
        if useObs
            addObserver(mdl, nm, A, B, C, L, x, y0, h);
            wire(mdl, ['Observer_' nm '/1'], ['Kx_' nm '/1']);
        else
            wire(mdl, ['Airframe_' nm '/1'], ['Kx_' nm '/1']);
        end
end
if useObs
    wire(mdl, ['Elevator_' nm '/1'], ['Obs_in_' nm '/1']);
    wire(mdl, ['Theta_' nm '/1'],    ['Obs_in_' nm '/2']);
end

% ---- degree-valued outputs ---------------------------------------------
blk(mdl, 'simulink/Math Operations/Gain', ['Theta_deg_' nm], pos(14,0,40), ...
    'Gain', num2str(180/pi, 10));
wire(mdl, ['Theta_' nm '/1'], ['Theta_deg_' nm '/1']);
blk(mdl, 'simulink/Sources/Constant', ['Trim_deg_' nm], pos(14,1,40), ...
    'Value', num2str(cfg.theta_trim_deg, 10));
blk(mdl, 'simulink/Math Operations/Sum', ['Attitude_abs_' nm], pos(15,0,30), 'Inputs', '++');
wire(mdl, ['Theta_deg_' nm '/1'], ['Attitude_abs_' nm '/1']);
wire(mdl, ['Trim_deg_' nm '/1'],  ['Attitude_abs_' nm '/2']);
blk(mdl, 'simulink/Sinks/To Workspace', ['Log_attitude_' nm], pos(16,0,60), ...
    'VariableName', ['theta_deg_' nm], 'SaveFormat', 'StructureWithTime');
wire(mdl, ['Attitude_abs_' nm '/1'], ['Log_attitude_' nm '/1']);

blk(mdl, 'simulink/Math Operations/Gain', ['U_deg_' nm], pos(14,2,40), ...
    'Gain', num2str(180/pi, 10));
wire(mdl, uCmd, ['U_deg_' nm '/1']);
blk(mdl, 'simulink/Sources/Constant', ['Elev_trim_deg_' nm], pos(14,3,50), ...
    'Value', num2str(cfg.delta_trim_deg, 10));
blk(mdl, 'simulink/Math Operations/Sum', ['Elevator_cmd_' nm], pos(15,2,30), 'Inputs', '++');
wire(mdl, ['U_deg_' nm '/1'], ['Elevator_cmd_' nm '/1']);
wire(mdl, ['Elev_trim_deg_' nm '/1'], ['Elevator_cmd_' nm '/2']);
blk(mdl, 'simulink/Sinks/To Workspace', ['Log_elevator_' nm], pos(16,2,60), ...
    'VariableName', ['delta_cmd_deg_' nm], 'SaveFormat', 'StructureWithTime');
wire(mdl, ['Elevator_cmd_' nm '/1'], ['Log_elevator_' nm '/1']);
end

% =========================================================================
function addObserver(mdl, nm, A, B, C, L, x, y0, h)
% Luenberger observer:  xhat_dot = (A - L C) xhat + [B L] [delta; theta]
blk(mdl, 'simulink/Signal Routing/Mux', ['Obs_in_' nm], ...
    [x(11), y0 + 145, x(11) + 5, y0 + 205], 'Inputs', '2');
add_block('simulink/Continuous/State-Space', [mdl '/Observer_' nm], ...
    'Position', [x(12), y0 + 140, x(12) + 70, y0 + 140 + h], ...
    'A', mat2str(A - L*C, 10), 'B', mat2str([B L], 10), ...
    'C', mat2str(eye(3)), 'D', mat2str(zeros(3,2)), 'X0', '[0;0;0]');
wire(mdl, ['Obs_in_' nm '/1'], ['Observer_' nm '/1']);
end

% =========================================================================
function blk(mdl, lib, name, pos, varargin)
add_block(lib, [mdl '/' name], 'Position', pos, varargin{:});
end

function wire(mdl, from, to)
add_line(mdl, from, to, 'autorouting', 'smart');
end
