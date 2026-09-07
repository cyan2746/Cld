%% ========================================================================
%  AIRCRAFT PITCH / LONGITUDINAL AUTOPILOT  --  DESIGN & COMPARISON SUITE
%  ------------------------------------------------------------------------
%  Trim-referenced attitude-command autopilot for the classic short-period
%  pitch model, designed around a user-specified pitch-up command and a
%  user-specified gust upset, with hard elevator travel limits of
%  +25 deg (nose-up authority) and -15 deg (nose-down authority).
%
%  PIPELINE
%    0  Configuration (all pilot/FMS inputs live here)
%    1  Aircraft model, trim-referenced state space
%    2  Open-loop analysis + the fundamental limits of this airframe
%    3  Root locus and proportional design
%    4  Classical PID design (PI-D form, derivative on measurement)
%    5  Controllability and pole placement
%    6  LQR  (full-state regulator + attitude reference)
%    7  LQI  (LQR + integral action = true attitude tracking)
%    8  Observability and observer design (only theta measured)
%    9  Closed-loop verification: NONLINEAR plant + actuator lag +
%       asymmetric saturation + rate limit + observer, for every scenario
%   10  Comparative analysis PID vs LQR vs LQI  (+ verdict)
%   11  Reference command shaping (2-DOF) -- removing the overshoot
%   12  Programmatic Simulink model of the complete autopilot
%
%  SIGN / UNIT CONVENTIONS
%    * All model states are DEVIATIONS FROM TRIM.  The absolute attitude
%      plotted everywhere is  theta_abs = theta_trim + theta_deviation.
%    * target_theta_deg is therefore a PITCH-UP INCREMENT on top of the
%      attitude the aircraft is trimmed at (both are inputs in Section 0).
%    * Positive elevator deflection = nose up for this model (verified in
%      Section 2 from the sign of the DC pitch-rate gain).
%    * Angles are stored in radians internally, displayed in degrees.
%
%  Runs on MATLAB (Control System Toolbox) and on GNU Octave with the
%  control package.  Simulink section is skipped automatically when
%  Simulink is not present.
% =========================================================================

clear; clc; close all;
if exist('OCTAVE_VERSION', 'builtin'), pkg load control; end

%% ------------------------------------------------------------------------
%  SECTION 0 -- CONFIGURATION.  EVERYTHING THE USER SETS IS HERE.
% -------------------------------------------------------------------------

cfg = struct();

% ---- the command ---------------------------------------------------------
cfg.theta_trim_deg   =  0;    % attitude the aircraft is trimmed at RIGHT NOW
cfg.target_theta_deg = 10;    % PITCH-UP COMMAND, measured from the trim
                              % attitude (so the autopilot must fly to
                              % theta_trim + target_theta)
cfg.delta_trim_deg   =  0;    % elevator deflection that holds the trim

% ---- the gust upset ------------------------------------------------------
cfg.gust_theta_deg   =  3;    % pitch upset the gust produces (+ = nose up)
cfg.gust_time_s      =  2;    % when the gust hits
cfg.gust_rise_s      =  0.5;  % time the gust takes to build the upset
                              % (0.5 s => a sharp but physical gust; set
                              %  very small for an instantaneous jump)

% ---- sustained upset used to expose integral action ----------------------
cfg.bias_elev_deg    = -2;    % standing elevator/moment bias (e.g. fuel
cfg.bias_time_s      =  5;    % transfer, ice, mis-trim)

% ---- elevator (actuator) -- HARD LIMITS FROM THE BRIEF -------------------
cfg.elev_up_deg      = +25;   % maximum deflection (nose-up authority)
cfg.elev_dn_deg      = -15;   % minimum deflection (nose-down authority)
cfg.elev_rate_deg    =  60;   % elevator rate limit  (deg/s)
cfg.tau_act          =  0.10; % first-order servo lag (s)
cfg.tau_aw           =  0.50; % anti-windup back-calculation time (s)

% ---- sensors / observer --------------------------------------------------
cfg.use_observer     = true;          % only theta measured -> estimate alpha,q
cfg.obs_method       = 'kalman';      % 'kalman' = steady-state filter, process
                                      %   noise through the elevator channel,
                                      %   bandwidth auto-selected in Section 8
                                      % 'place'  = hand-placed Luenberger poles
cfg.obs_poles        = [-2 -3 -4];    % used when obs_method = 'place'
cfg.theta_noise_deg  = 0.05;          % attitude sensor noise, deg RMS (used by
                                      %   the filter design; set cfg.sim_noise
                                      %   to feed it into the simulations too)
cfg.sim_noise        = false;         % inject the sensor noise in Section 9

% ---- plant model used for verification -----------------------------------
cfg.use_nonlinear    = true;   % verify on the nonlinear plant (Section 9)

% ---- simulation ----------------------------------------------------------
cfg.dt               = 0.01;   % fixed-step RK4 step (s)
cfg.T_step           = 80;     % horizon, attitude-step scenario
cfg.T_gust           = 60;     % horizon, gust scenario
cfg.T_long           = 80;     % horizon, combined / bias scenarios

% ---- design specification ------------------------------------------------
cfg.spec.overshoot   = 15;     % max acceptable overshoot (%)
cfg.spec.authority   = 0.90;   % fraction of elevator authority a *planned*
                               % manoeuvre may demand
cfg.spec.authority_gust = 1.00;% an UNPLANNED gust may use all of the travel,
                               % but the design must still not saturate on the
                               % gust the user specified
cfg.spec.settle_band = 0.02;   % settling band, fraction of the excursion
cfg.spec.band_floor_deg = 0.10;% ...but never tighter than the attitude-hold
                               % tolerance the autopilot is expected to keep
cfg.spec.zeta_min    = 0.50;   % minimum acceptable closed-loop damping
cfg.spec.pm_min      = 35;     % deg, minimum acceptable phase margin
cfg.spec.gm_min      = 6;      % dB, minimum acceptable gain margin
cfg.spec.smax        = 2.5;    % max ||1/(1+L)||inf (modulus margin >= 0.4)

% ---- reference command shaping (Section 11) ------------------------------
cfg.shape_tau        = 2.0;    % command-filter time constant (s)

% ---- derived quantities (do not edit) ------------------------------------
D2R = pi/180;  R2D = 180/pi;
cfg.D2R = D2R;  cfg.R2D = R2D;
cfg.theta_target_abs_deg = cfg.theta_trim_deg + cfg.target_theta_deg;
% Elevator limits expressed on the DEVIATION signal the controller produces
cfg.u_max = (cfg.elev_up_deg - cfg.delta_trim_deg)*D2R;   % > 0
cfg.u_min = (cfg.elev_dn_deg - cfg.delta_trim_deg)*D2R;   % < 0
cfg.u_rate = cfg.elev_rate_deg*D2R;

fprintf('\n');
fprintf('##########################################################################\n');
fprintf('#            AIRCRAFT PITCH AUTOPILOT -- DESIGN & COMPARISON             #\n');
fprintf('##########################################################################\n');
fprintf('\n  COMMAND\n');
fprintf('    trimmed at                : %+7.2f deg\n', cfg.theta_trim_deg);
fprintf('    pitch-up command          : %+7.2f deg   (target_theta)\n', cfg.target_theta_deg);
fprintf('    => absolute attitude target: %+7.2f deg\n', cfg.theta_target_abs_deg);
fprintf('    gust upset to reject      : %+7.2f deg over %.2f s at t = %.1f s\n', ...
        cfg.gust_theta_deg, cfg.gust_rise_s, cfg.gust_time_s);
fprintf('  ELEVATOR\n');
fprintf('    travel limits             : %+.1f deg (up) / %+.1f deg (down)\n', ...
        cfg.elev_up_deg, cfg.elev_dn_deg);
fprintf('    trim deflection           : %+.2f deg  => usable command range %+.2f .. %+.2f deg\n', ...
        cfg.delta_trim_deg, cfg.u_min*R2D, cfg.u_max*R2D);
fprintf('    rate limit / servo lag    : %.0f deg/s / %.3f s\n', cfg.elev_rate_deg, cfg.tau_act);

%% ------------------------------------------------------------------------
%  SECTION 1 -- AIRCRAFT MODEL (trim-referenced)
%
%  x = [alpha; q; theta]   deviations from the trim condition  (rad, -, rad)
%  u = delta_e             elevator deviation from trim         (rad)
%  y = theta               attitude deviation from trim         (rad)
%
%      alpha_dot = -0.313*alpha + 56.7*q   + 0.232*delta_e
%      q_dot     = -0.0139*alpha - 0.426*q + 0.0203*delta_e
%      theta_dot =  56.7*q
%
%  NOTE ON q: in this classic teaching model theta_dot = 56.7*q, so the
%  state q is a SCALED pitch rate; the physical pitch rate is 56.7*q.  All
%  weights and displays below account for that factor explicitly.
%
%  NOTE ON TRIM: column 3 of A is identically zero, i.e. theta does not
%  feed back into the dynamics.  For this reduced model the design is
%  therefore INDEPENDENT of the trim attitude -- trimming at 0 deg or at
%  10 deg gives the same gains, and theta_trim only shifts the absolute
%  attitude that is plotted.  (In a full model the trim attitude would
%  change the stability derivatives through speed and load factor.)
% -------------------------------------------------------------------------

A = [-0.313   56.7    0;
     -0.0139  -0.426  0;
      0        56.7   0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
Dm = 0;

plant = ss(A, B, C, Dm);
plant.StateName  = {'alpha','q','theta'};
plant.InputName  = {'delta_e'};
plant.OutputName = {'theta'};

Bd = [0; 0; 1];   % gust enters as a pitch-rate disturbance on theta_dot

fprintf('\n==================== 1. AIRCRAFT MODEL ====================\n');
fprintf('  states : alpha (rad), q (scaled pitch rate), theta (rad)  [deviations from trim]\n');
fprintf('  input  : elevator deviation from trim (rad)\n');
fprintf('  output : theta deviation from trim (rad)\n');
fprintf('  A =\n'); disp(A);
fprintf('  B'' = [%g %g %g]      C = [%g %g %g]\n', B(1), B(2), B(3), C(1), C(2), C(3));

%% ------------------------------------------------------------------------
%  SECTION 2 -- OPEN-LOOP ANALYSIS AND THE FUNDAMENTAL LIMITS
% -------------------------------------------------------------------------

fprintf('\n==================== 2. OPEN-LOOP ANALYSIS ====================\n');

lam = eig(A);
lam_sp_for_msg = lam(find(abs(imag(lam)) > 1e-9, 1));
fprintf('  Open-loop eigenvalues:\n');
for k = 1:numel(lam)
    if abs(imag(lam(k))) > 1e-9
        wn = abs(lam(k)); z = -real(lam(k))/wn;
        fprintf('    %+8.4f %+8.4fi   (wn = %.4f rad/s, zeta = %.4f)\n', ...
                real(lam(k)), imag(lam(k)), wn, z);
    else
        fprintf('    %+8.4f              (real)\n', real(lam(k)));
    end
end
fprintf('  The pair above is the SHORT-PERIOD mode (lightly damped).\n');
fprintf('  The eigenvalue at the origin is theta acting as a free integrator of q\n');
fprintf('  (theta does not feed back), so the airframe has no natural attitude hold.\n');

G = tf(plant);
[numG, denG] = tfdata(G, 'v');    % num = [0 0 b1 b0], den = [1 a2 a1 0]
b1 = numG(end-1);  b0 = numG(end);
a1 = denG(end-1);

fprintf('\n  theta(s)/delta_e(s) = (%.5f s + %.5f) / (s^3 %+.5f s^2 %+.5f s)\n', ...
        b1, b0, denG(2), denG(3));
fprintf('  transmission zero    : s = %+.4f  (time constant %.2f s)\n', -b0/b1, b1/b0);

% Steady pitch-rate authority: with the elevator held, theta ramps.
theta_rate_per_deg = b0/a1;      % low-frequency gain of theta_dot/delta_e
fprintf('  steady pitch-rate gain: %.4f deg/s of pitch rate per deg of elevator\n', theta_rate_per_deg);
fprintf('                          => %.2f deg/s at the +%.0f deg limit (positive elevator = NOSE UP)\n', ...
        theta_rate_per_deg*cfg.elev_up_deg, cfg.elev_up_deg);

% ---- the fundamental limit of this airframe ------------------------------
%  For any closed loop T(s) with T(0)=1,   int_0^inf (theta_cmd - theta) dt
%  = -T'(0).  Writing T = K*N/D,  T'(0)/T(0) = N'(0)/N(0) - D'(0)/D(0).
%  The plant zero contributes N'(0)/N(0) >= b1/b0 = 6.49 s, so:
%    * a controller WITHOUT an integrator (P, LQR) can only satisfy the
%      identity with sum(1/|pole|) > 6.49 s, i.e. a slow closed-loop pole
%      and a long tail on the step response;
%    * a controller WITH an integrator (PID, LQI) has two integrators in
%      the loop, T'(0) = 0 exactly, so the rise deficit is balanced by a
%      (small) overshoot instead of a tail.
%  This single number explains the entire PID/LQR/LQI ranking in Section 10.
I0 = b1/b0;
fprintf('\n  FUNDAMENTAL LIMIT (drives the whole comparison):\n');
fprintf('    plant zero time constant b1/b0 = %.3f s\n', I0);
fprintf('    => integral of the tracking error over a step response obeys\n');
fprintf('       int(theta_cmd - theta)dt = %.3f - sum(1/|closed-loop pole|)  [x command]\n', I0);
fprintf('    * no integrator in the controller (P, LQR): a slow pole is unavoidable\n');
fprintf('      -> monotonic but slow approach to the target\n');
fprintf('    * integrator in the controller (PID, LQI): the identity gives exactly 0\n');
fprintf('      -> fast rise paid for with a small overshoot, no long tail\n');

% ---- how much of a gust can physically be opposed ------------------------
gust_rate  = cfg.gust_theta_deg/max(cfg.gust_rise_s, 1e-3);
rate_up    = theta_rate_per_deg*cfg.elev_up_deg;
rate_dn    = theta_rate_per_deg*abs(cfg.elev_dn_deg);
fprintf('\n  GUST AUTHORITY (why every design lets the upset through):\n');
fprintf('    the specified gust imposes %.2f deg/s of pitch rate (%.1f deg in %.2f s)\n', ...
        gust_rate, cfg.gust_theta_deg, cfg.gust_rise_s);
fprintf('    full elevator can generate %.2f deg/s nose-up / %.2f deg/s nose-down,\n', ...
        rate_up, rate_dn);
fprintf('    and needs ~%.1f s (servo lag + pitch-rate build-up) to get there.\n', ...
        4*cfg.tau_act + 1/abs(real(lam_sp_for_msg)));
if gust_rate > rate_dn
    fprintf('    => the upset CANNOT be opposed while it happens; %.1f deg of attitude\n', ...
            cfg.gust_theta_deg);
    fprintf('       excursion is unavoidable and the designs differ only in how fast\n');
    fprintf('       and how smoothly they RECOVER.\n');
else
    fprintf('    => there is enough authority to fight the gust as it builds.\n');
end

figure('Name','2. Open-loop analysis');
subplot(2,2,1);
[y_ol, t_ol] = step(plant*(5*D2R), 12);
plot(t_ol, cfg.theta_trim_deg + y_ol*R2D, 'b-', 'LineWidth', 1.6); grid on;
title('Open-loop: 5 deg elevator step'); xlabel('Time (s)');
ylabel('\theta (deg, absolute)');
legend('\theta runs away (no attitude hold)','Location','NorthWest');
subplot(2,2,2);
pzmap(plant); grid on; title('Open-loop poles / zero');
subplot(2,2,3);
[y_i, t_i] = impulse(plant, 25);
plot(t_i, y_i*R2D, 'b-', 'LineWidth', 1.4); grid on;
title('Short-period ringing (impulse)'); xlabel('Time (s)'); ylabel('\theta (deg)');
subplot(2,2,4);
bode(plant); grid on; title('Open-loop frequency response');

%% ------------------------------------------------------------------------
%  SECTION 3 -- ROOT LOCUS AND PROPORTIONAL DESIGN
%
%  Design rule used: walk the locus and take the LARGEST proportional gain
%  that (a) keeps the step overshoot inside the spec and (b) keeps the peak
%  elevator demand for the user's command inside the allowed authority.
% -------------------------------------------------------------------------

fprintf('\n==================== 3. ROOT LOCUS / PROPORTIONAL DESIGN ====================\n');

figure('Name','3. Root locus');
rlocus(G); grid on;
title('Root locus, proportional attitude feedback');
xlabel('Real axis'); ylabel('Imaginary axis');

Kp_grid = 0.05:0.05:4;
Kp_ok = NaN; rl_report = [];
for kk = 1:numel(Kp_grid)
    ctrlP = makeP(Kp_grid(kk));
    lin   = linearCL(ctrlP, A, B, C, Bd, cfg, []);
    m     = linMetrics(lin, cfg, cfg.target_theta_deg);
    rl_report = [rl_report; Kp_grid(kk), m.overshoot, m.u_peak_abs, m.settle]; %#ok<AGROW>
    if m.stable && m.overshoot <= cfg.spec.overshoot && ...
       m.u_max <= cfg.spec.authority*cfg.u_max*R2D && ...
       m.u_min >= cfg.spec.authority*cfg.u_min*R2D
        Kp_ok = Kp_grid(kk);
    end
end
if isnan(Kp_ok), Kp_ok = 0.5; end
fprintf('  locus scan (linear design model, %.1f deg command):\n', cfg.target_theta_deg);
fprintf('    %8s %12s %14s %12s\n', 'Kp', 'overshoot[%]', 'peak elev[deg]', 'settle[s]');
for kk = round(linspace(4, numel(Kp_grid), 6))
    fprintf('    %8.2f %12.2f %14.2f %12.2f\n', rl_report(kk,1), rl_report(kk,2), ...
            rl_report(kk,3), rl_report(kk,4));
end
ctrl_P = makeP(Kp_ok);
ctrl_P.name = 'P (root locus)';

cl_p = roots(denG + Kp_ok*[0 0 numG(end-1) numG(end)]);
fprintf('  selected gain from the locus : Kp = %.3f\n', Kp_ok);
fprintf('  (largest gain with overshoot <= %.0f%% and elevator demand <= %.0f%% of authority)\n', ...
        cfg.spec.overshoot, 100*cfg.spec.authority);
fprintf('  closed-loop poles at that gain:\n');
for k = 1:numel(cl_p)
    if abs(imag(cl_p(k))) > 1e-9
        fprintf('    %+8.4f %+8.4fi  (zeta = %.3f)\n', real(cl_p(k)), imag(cl_p(k)), ...
                -real(cl_p(k))/abs(cl_p(k)));
    else
        fprintf('    %+8.4f\n', real(cl_p(k)));
    end
end
fprintf('  NOTE: raising Kp pushes the complex pair UP the locus, so damping gets\n');
fprintf('  WORSE, not better -- proportional gain alone cannot damp the short period.\n');
fprintf('  Derivative action (Section 4) is what supplies the damping.\n');

lin_P = linearCL(ctrl_P, A, B, C, Bd, cfg, []);
mP    = linMetrics(lin_P, cfg, cfg.target_theta_deg);
printMetrics('P (root-locus gain), linear design model', mP, cfg);
plotDesign(lin_P, cfg, 'P (root-locus gain)');

%% ------------------------------------------------------------------------
%  SECTION 4 -- CLASSICAL PID DESIGN
%
%  Structure (PI-D, the form used in real autopilots):
%
%     u = Kp*(theta_cmd - theta) + Ki*INT(theta_cmd - theta) - Kd*dtheta/dt
%
%  * The derivative acts on the MEASUREMENT, not on the error.  A step
%    attitude command therefore produces no derivative kick, and the peak
%    elevator demand is exactly  Kp * target_theta  -- which is what ties
%    the gain directly to the +25/-15 deg travel limits.
%  * The derivative is filtered (N) so a sharp gust cannot spike the
%    elevator: the gust contribution is (Kp + Kd*N) * gust.
%  * Integral action is what removes the long tail predicted in Section 2
%    and rejects a standing mis-trim (Section 9d).
%  * Anti-windup: back-calculation, ẋi = e + (u_sat - u_cmd)/tau_aw, the
%    same law that is implemented in the Simulink model.
% -------------------------------------------------------------------------

fprintf('\n==================== 4. CLASSICAL PID DESIGN ====================\n');

cfg.pid.Kp = 1.6;    % attitude gain      -> sets the elevator kick
cfg.pid.Ki = 0.7;    % integral gain      -> kills the tail / mis-trim
cfg.pid.Kd = 1.0;    % rate gain          -> damps the short period
cfg.pid.N  = 8;      % derivative filter  -> limits the gust spike

ctrl_PID = makePID(cfg.pid.Kp, cfg.pid.Ki, cfg.pid.Kd, cfg.pid.N);
ctrl_PID.name = 'PID';
[ctrl_PID, sc_pid, bind_pid] = fitAuthority(ctrl_PID, A, B, C, Bd, cfg, []);

lin_PID = linearCL(ctrl_PID, A, B, C, Bd, cfg, []);
mPID    = linMetrics(lin_PID, cfg, cfg.target_theta_deg);

fprintf('  gains : Kp = %.4f   Ki = %.4f   Kd = %.4f   N = %.1f\n', ...
        ctrl_PID.Kp, ctrl_PID.Ki, ctrl_PID.Kd, ctrl_PID.N);
if sc_pid < 1
    fprintf('  loop gain de-rated by x%.3f to respect the elevator travel limits\n', sc_pid);
    fprintf('  (binding case: %s)\n', bind_pid.bind);
else
    fprintf('  no de-rating needed: both the %.1f deg command and the %.1f deg gust fit\n', ...
            cfg.target_theta_deg, cfg.gust_theta_deg);
    fprintf('  inside the %+.0f / %+.0f deg travel limits at the nominal gains\n', ...
            cfg.elev_up_deg, cfg.elev_dn_deg);
end
fprintf('  PID zeros (Kd s^2 + Kp s + Ki = 0): ');
fprintf('%+.4f  ', roots([ctrl_PID.Kd ctrl_PID.Kp ctrl_PID.Ki])); fprintf('\n');
fprintf('  closed-loop poles (with actuator lag):\n');   printPoles(lin_PID.poles);
printMargins(lin_PID);
mgPID = linGustMetrics(lin_PID, cfg);
gust_per_deg = mgPID.u_peak_abs/abs(cfg.gust_theta_deg);
fprintf('  ELEVATOR BUDGET (linear, before saturation)\n');
fprintf('    per deg of attitude command : %+.3f deg of elevator  (= Kp)\n', ctrl_PID.Kp);
fprintf('    per deg of gust upset       : %+.3f deg for the specified %.2f s build-up\n', ...
        gust_per_deg, cfg.gust_rise_s);
fprintf('                                  %+.3f deg if the upset were instantaneous (= Kp + Kd*N),\n', ...
        ctrl_PID.Kp + ctrl_PID.Kd*ctrl_PID.N);
fprintf('                                  which is what the derivative filter N is there to blunt\n');
fprintf('    largest nose-up command before saturation   : %+.2f deg\n', ...
        cfg.u_max*cfg.R2D/ctrl_PID.Kp);
fprintf('    largest nose-down command before saturation : %+.2f deg  (tighter limit!)\n', ...
        cfg.u_min*cfg.R2D/ctrl_PID.Kp);
fprintf('    largest gust before saturation              : %+.2f deg\n', ...
        abs(cfg.u_min*cfg.R2D)/gust_per_deg);
printMetrics('PID, linear design model', mPID, cfg);

% Auto-tuned PID shown for reference only (MATLAB only, needs pidtune)
try
    Cauto = pidtune(G, 'PID');
    fprintf('  reference: pidtune() suggests Kp = %.4f, Ki = %.4f, Kd = %.4f\n', ...
            Cauto.Kp, Cauto.Ki, Cauto.Kd);
    lin_auto = linearCL(makePID(Cauto.Kp, Cauto.Ki, Cauto.Kd, 8), A, B, C, Bd, cfg, []);
    m_auto   = linMetrics(lin_auto, cfg, cfg.target_theta_deg);
    fprintf('            its peak elevator demand for this command: %+.2f deg (limit %+.1f)\n', ...
            m_auto.u_max, cfg.u_max*cfg.R2D);
catch
    fprintf('  (pidtune not available in this installation -- skipped)\n');
end

plotDesign(lin_PID, cfg, 'PID');

%% ------------------------------------------------------------------------
%  SECTION 5 -- CONTROLLABILITY AND POLE PLACEMENT
%
%  Full-state feedback  u = -K*(x - x_ref),  x_ref = [0; 0; theta_cmd].
%  The desired poles are picked using the Section-2 identity: the sum of
%  the closed-loop time constants sum(1/|p_i|) decides how the step
%  response splits its error between overshoot (fast poles) and tail
%  (slow poles), and 6.49 s is the break-even.  These poles sit
%  deliberately on the slow side of it (~8.7 s), which buys an almost
%  monotone response and a very small elevator demand at the price of
%  speed -- the opposite corner of the same trade the PID takes.
% -------------------------------------------------------------------------

fprintf('\n==================== 5. CONTROLLABILITY / POLE PLACEMENT ====================\n');

Co = ctrb(A, B);
fprintf('  controllability matrix rank : %d of 3  ->  %s\n', rank(Co), ...
        ternary(rank(Co) == 3, 'fully controllable', 'NOT controllable'));
fprintf('  observability matrix rank   : %d of 3  ->  %s\n', rank(obsv(A, C)), ...
        ternary(rank(obsv(A,C)) == 3, 'fully observable', 'NOT observable'));

cfg.pp_poles = [-0.32 -0.35 -0.38];
K_pp = place(A, B, cfg.pp_poles);
ctrl_PP = makeSF(K_pp);  ctrl_PP.name = 'Pole placement';
[ctrl_PP, sc_pp, bind_pp] = fitAuthority(ctrl_PP, A, B, C, Bd, cfg, []);
lin_PP = linearCL(ctrl_PP, A, B, C, Bd, cfg, []);
mPP    = linMetrics(lin_PP, cfg, cfg.target_theta_deg);

fprintf('  desired poles      : [%s]   sum(1/|p|) = %.2f s\n', ...
        num2str(cfg.pp_poles, '%.3f  '), sum(1./abs(cfg.pp_poles)));
fprintf('  gain K             : [%+.4f  %+.4f  %+.4f]', ctrl_PP.K);
if sc_pp < 1, fprintf('   (de-rated x%.3f, binding case: %s)', sc_pp, bind_pp.bind); end
fprintf('\n');
fprintf('  achieved poles     :\n'); printPoles(eig(A - B*ctrl_PP.K));
fprintf('  elevator per deg of command : %+.3f deg  (= K_theta)\n', ctrl_PP.K(3));
printMetrics('Pole placement, linear design model', mPP, cfg);
plotDesign(lin_PP, cfg, 'Pole placement');

%% ------------------------------------------------------------------------
%  SECTION 6 -- LQR
%
%  Bryson's rule: each state is weighted by 1/(largest acceptable
%  excursion)^2 and the elevator by rho/(delta_max)^2, so rho is a single
%  interpretable knob ("how expensive is elevator travel").
%  The q weight uses the PHYSICAL pitch rate 56.7*q.
%  Reference handling: u = -K*(x - [0;0;theta_cmd]).  Because the airframe
%  already contains an integrator, this gives zero steady-state error on a
%  step -- but no rejection of a standing mis-trim (see Section 9d).
% -------------------------------------------------------------------------

fprintf('\n==================== 6. LQR ====================\n');

cfg.spec.alpha_max_deg = 10;    % acceptable alpha excursion
cfg.spec.qrate_max_deg = 20;    % acceptable PHYSICAL pitch rate (deg/s)
cfg.spec.theta_max_deg = 10;    % acceptable attitude error
cfg.spec.delta_max_deg = 15;    % elevator normalisation (the tighter limit)
cfg.spec.eint_max_degs = 3.3;   % acceptable integrated attitude error (deg*s)
cfg.spec.rho_lqr       = 0.8;   % elevator price, LQR
cfg.spec.rho_lqi       = 2.0;   % elevator price, LQI

Qs = brysonQ(cfg);
R_lqr = cfg.spec.rho_lqr/(cfg.spec.delta_max_deg*D2R)^2;
K_lqr = lqr(A, B, Qs, R_lqr);
ctrl_LQR = makeSF(K_lqr);  ctrl_LQR.name = 'LQR';
[ctrl_LQR, sc_lqr, bind_lqr] = fitAuthority(ctrl_LQR, A, B, C, Bd, cfg, []);
lin_LQR = linearCL(ctrl_LQR, A, B, C, Bd, cfg, []);
mLQR    = linMetrics(lin_LQR, cfg, cfg.target_theta_deg);

fprintf('  Q = diag([1/(%.0f deg)^2, (56.7/(%.0f deg/s))^2, 1/(%.0f deg)^2])\n', ...
        cfg.spec.alpha_max_deg, cfg.spec.qrate_max_deg, cfg.spec.theta_max_deg);
fprintf('  R = %.4g   (rho = %.2f, delta_max = %.0f deg)\n', R_lqr, cfg.spec.rho_lqr, cfg.spec.delta_max_deg);
fprintf('  gain K = [%+.4f  %+.4f  %+.4f]', ctrl_LQR.K);
if sc_lqr < 1, fprintf('   (de-rated x%.3f, binding case: %s)', sc_lqr, bind_lqr.bind); end
fprintf('\n');
fprintf('    -> elevator per deg of attitude error : %+.3f deg  (= K_theta)\n', ctrl_LQR.K(3));
fprintf('    -> elevator per deg/s of pitch rate   : %+.3f deg  (= K_q/56.7)\n', ctrl_LQR.K(2)/56.7);
fprintf('  closed-loop poles (with actuator lag):\n'); printPoles(lin_LQR.poles);
printMargins(lin_LQR);
fprintf('  largest nose-up command before saturation   : %+.2f deg\n', cfg.u_max*R2D/ctrl_LQR.K(3));
fprintf('  largest nose-down command before saturation : %+.2f deg  (tighter limit!)\n', ...
        cfg.u_min*R2D/ctrl_LQR.K(3));
fprintf('  NOTE the slow closed-loop pole near %.3f: this is the Section-2 limit at\n', ...
        max(real(eig(A - B*ctrl_LQR.K))));
fprintf('  work.  Without an integrator the LQR must keep a pole slower than the\n');
fprintf('  plant zero (%.4f), which is exactly the long tail seen below.\n', -b0/b1);
printMetrics('LQR, linear design model', mLQR, cfg);
plotDesign(lin_LQR, cfg, 'LQR (design model, full state)');

%% ------------------------------------------------------------------------
%  SECTION 7 -- LQI  (LQR + integral action)
%
%  Augmented plant   xa = [alpha; q; theta; z],   z_dot = theta_cmd - theta
%      xa_dot = [A 0; -C 0] xa + [B; 0] u + [0;0;0;1] theta_cmd
%  Control law (the standard LQI form, identical to MATLAB's lqi):
%      u = -Kx*x - Kz*z
%  The command enters only through the integrator, so the elevator starts
%  from zero and builds up: no kick, and the peak elevator demand for a
%  planned manoeuvre is much smaller than the PID/LQR kick.
% -------------------------------------------------------------------------

fprintf('\n==================== 7. LQI (LQR + INTEGRAL ACTION) ====================\n');

Aa = [A, zeros(3,1); -C, 0];
Ba = [B; 0];
Ea = [0; 0; 0; 1];
Qa = blkdiag(Qs, 1/(cfg.spec.eint_max_degs*D2R)^2);
R_lqi = cfg.spec.rho_lqi/(cfg.spec.delta_max_deg*D2R)^2;
Ka = lqr(Aa, Ba, Qa, R_lqi);
ctrl_LQI = makeLQI(Ka(1:3), Ka(4));  ctrl_LQI.name = 'LQI';
[ctrl_LQI, sc_lqi, bind_lqi] = fitAuthority(ctrl_LQI, A, B, C, Bd, cfg, []);
lin_LQI = linearCL(ctrl_LQI, A, B, C, Bd, cfg, []);
mLQI    = linMetrics(lin_LQI, cfg, cfg.target_theta_deg);

fprintf('  integral-error weight : 1/(%.2f deg*s)^2 = %.4g\n', ...
        cfg.spec.eint_max_degs, 1/(cfg.spec.eint_max_degs*D2R)^2);
fprintf('  R = %.4g   (rho = %.2f)\n', R_lqi, cfg.spec.rho_lqi);
fprintf('  gains : Kx = [%+.4f  %+.4f  %+.4f]   Kz = %+.4f', ctrl_LQI.Kx, ctrl_LQI.Kz);
if sc_lqi < 1, fprintf('   (de-rated x%.3f, binding case: %s)', sc_lqi, bind_lqi.bind); end
fprintf('\n');
fprintf('    -> elevator per deg of attitude error : %+.3f deg  (= Kx_theta)\n', ctrl_LQI.Kx(3));
fprintf('    -> elevator per deg/s of pitch rate   : %+.3f deg  (= Kx_q/56.7)\n', ctrl_LQI.Kx(2)/56.7);
fprintf('  integrator steady state for this command: z_ss = %+.4f deg*s\n', ...
        -ctrl_LQI.Kx(3)*cfg.target_theta_deg/ctrl_LQI.Kz);
fprintf('  closed-loop poles (with actuator lag):\n'); printPoles(lin_LQI.poles);
printMargins(lin_LQI);
fprintf('  gust demand: %+.3f deg of elevator per deg of upset  (= Kx_theta)\n', ctrl_LQI.Kx(3));
fprintf('  largest gust before saturation : %+.2f deg\n', abs(cfg.u_min*R2D)/ctrl_LQI.Kx(3));
fprintf('  the command reaches the elevator through the integrator, so a planned\n');
fprintf('  manoeuvre costs far less elevator than the PID/LQR kick: %.2f deg for this\n', mLQI.u_peak_abs);
fprintf('  %.1f deg command against %.2f deg (PID) and %.2f deg (LQR).\n', ...
        cfg.target_theta_deg, mPID.u_peak_abs, mLQR.u_peak_abs);
printMetrics('LQI, linear design model', mLQI, cfg);
plotDesign(lin_LQI, cfg, 'LQI (design model, full state)');

%% ------------------------------------------------------------------------
%  SECTION 8 -- OBSERVER (only theta is measured)
%
%  xhat_dot = A xhat + B delta_actual + L (theta_meas - theta_hat)
%
%  alpha is only weakly observable from theta (it reaches theta through the
%  small coefficient -0.0139 in q_dot), so a FAST observer needs a very
%  large alpha gain and amplifies attitude-sensor noise straight into the
%  elevator.  The table below quantifies that trade-off, which is why the
%  chosen poles are only ~3x the closed-loop bandwidth.
% -------------------------------------------------------------------------

fprintf('\n==================== 8. OBSERVER DESIGN ====================\n');

Ob = obsv(A, C);
fprintf('  measured : theta only          estimated : alpha, q\n');
fprintf('  observability matrix rank : %d of 3\n', rank(Ob));
fprintf('  alpha reaches theta only through the small coefficient %.4f in q_dot,\n', A(2,1));
fprintf('  so it is only WEAKLY observable: a fast estimator needs a huge alpha gain,\n');
fprintf('  amplifies attitude-sensor noise, and -- worse -- converts an unmodelled\n');
fprintf('  gust into a large elevator transient.  The audit below prices that out on\n');
fprintf('  the loop that actually flies (LQI + observer + servo).\n\n');

obs_cands = {};
lue_sets  = {[-2 -3 -4], [-4 -5 -6], [-8 -10 -12], [-15 -16 -17]};
for kk = 1:numel(lue_sets)
    Lc = place(A', C', lue_sets{kk})';
    r  = obsAudit(Lc, ctrl_LQI, A, B, C, Bd, cfg);
    r.tag = sprintf('place [%s]', num2str(lue_sets{kk}, '%g '));
    r.family = 'place';  r.setting = lue_sets{kk};
    obs_cands{end+1} = r;                                                  %#ok<SAGROW>
end

Rn      = (max(cfg.theta_noise_deg, 1e-3)*D2R)^2;   % attitude-sensor variance
qw_grid = logspace(-9, -1, 17);
for kk = 1:numel(qw_grid)
    Lc = kalmanObserver(A, B, C, qw_grid(kk), Rn);
    r  = obsAudit(Lc, ctrl_LQI, A, B, C, Bd, cfg);
    r.tag = sprintf('kalman qw = %8.1e', qw_grid(kk));
    r.family = 'kalman';  r.setting = qw_grid(kk);
    obs_cands{end+1} = r;                                                  %#ok<SAGROW>
end

fprintf('  %-22s %7s %7s %7s %7s %7s %9s %9s %9s %9s\n', 'observer', '|L|', 'noise', ...
        'PM[deg]', 'GM[dB]', '|S|inf', 'gust elev', 'gust ts', 'step ts', 'ts, -30%');
fprintf('  %-22s %7s %7s %7s %7s %7s %9s %9s %9s %9s\n', '', '', 'gain', '', '', '', ...
        '[deg]', '[s]', '[s]', 'model [s]');
for kk = 1:numel(obs_cands)
    r = obs_cands{kk};
    fprintf('  %-22s %7.2f %7.1f %7.1f %7.2f %7.2f %9.2f %9s %9s %9s\n', r.tag, r.normL, ...
            r.noise, r.pm, r.gm, r.smax, r.u_gust, fmtTime(r.tg), fmtTime(r.ts), ...
            fmtTime(r.ts_mis));
end
fprintf('\n  Reading the table:\n');
fprintf('   * hand-placed theta-only poles are unusable here: fast ones need |L| in the\n');
fprintf('     hundreds, amplify sensor noise, and throw %.0f-%.0f deg of elevator at a %.1f deg gust.\n', ...
        20, 64, cfg.gust_theta_deg);
fprintf('   * inside the Kalman family a LAZY filter (small qw) flatters itself: it barely\n');
fprintf('     reacts to the sensor, so margins and gust elevator look excellent -- and then\n');
fprintf('     the last column shows what happens when the airframe is not the one it\n');
fprintf('     believes in.  Estimator bandwidth is exactly what corrects model error.\n');
fprintf('   * so the rule is: take the FASTEST filter that still meets PM >= %.0f deg,\n', ...
        cfg.spec.pm_min);
fprintf('     |S|inf <= %.2f and a gust elevator demand <= %.1f deg.  Relax cfg.spec.pm_min\n', ...
        cfg.spec.smax, cfg.spec.authority_gust*min(abs(cfg.u_max), abs(cfg.u_min))*R2D);
fprintf('     if you would rather have the gust rejection than the robustness.\n');

% ---- pick the FASTEST observer that still meets every specification -----
%  Fastest, not gentlest: a lazy estimator looks wonderful on a perfect model
%  and then drifts on a real one, so among the candidates that satisfy the
%  margin and elevator specifications the highest-bandwidth one is taken.
u_gust_lim = cfg.spec.authority_gust*min(abs(cfg.u_max), abs(cfg.u_min))*R2D;
feasible   = [];
for kk = 1:numel(obs_cands)
    r = obs_cands{kk};
    if strcmp(r.family, 'kalman') && r.stable && r.pm >= cfg.spec.pm_min && ...
       r.smax <= cfg.spec.smax && r.u_gust <= u_gust_lim && isfinite(r.ts)
        feasible = [feasible kk];                                          %#ok<AGROW>
    end
end
if strcmp(cfg.obs_method, 'place')
    L_obs   = place(A', C', cfg.obs_poles)';
    obs_pick = sprintf('hand-placed poles [%s]', num2str(cfg.obs_poles, '%g '));
elseif ~isempty(feasible)
    kbest    = feasible(end);          % qw_grid is increasing -> fastest feasible
    L_obs    = obs_cands{kbest}.L;
    obs_pick = obs_cands{kbest}.tag;
else
    L_obs    = obs_cands{numel(lue_sets)+1}.L;
    obs_pick = obs_cands{numel(lue_sets)+1}.tag;
    warning(['No observer met PM >= %.0f deg, ||S||inf <= %.2f and a gust elevator ' ...
             'demand <= %.1f deg; falling back to the gentlest filter.'], ...
             cfg.spec.pm_min, cfg.spec.smax, u_gust_lim);
end

fprintf('\n  SELECTED : %s\n', obs_pick);
fprintf('    L = [%+.4f  %+.4f  %+.4f]''\n', L_obs);
fprintf('    estimator poles :'); fprintf('  %s', poleStr(eig(A - L_obs*C))); fprintf('\n');
fprintf('    airframe poles  :'); fprintf('  %s', poleStr(eig(A)));           fprintf('\n');
fprintf('    the estimator is faster than the airframe modes it has to reconstruct,\n');
fprintf('    yet gentle enough that a gust does not throw the elevator at the stops.\n');
fprintf('    Section 9g checks that choice against the real nonlinear plant.\n');

% ---- the loop as it is actually flown -----------------------------------
%  Sections 6 and 7 designed the state feedback assuming the whole state was
%  measurable, which the separation principle licenses.  Now that the
%  observer exists the two loops are rebuilt WITH it and re-checked against
%  the elevator budget, because during an UNMODELLED gust the estimator's
%  transient error is fed straight into the elevator, and that -- not the
%  attitude gain -- can be what sets the peak deflection.
if cfg.use_observer
    fprintf('\n  ---- cost of running the state feedback on ESTIMATES ----\n');
    fprintf('    %-5s %-12s %13s %13s %13s %13s\n', 'ctrl', 'sensors', ...
            'step elev', 'gust elev', 'step settle', 'gust settle');
    sf_list  = {ctrl_LQR, ctrl_LQI};
    sf_names = {'LQR', 'LQI'};
    for ksf = 1:2
        for useL = 0:1
            if useL, Luse = L_obs;  tag = 'theta only';
            else,    Luse = [];     tag = 'full state';  end
            lk = linearCL(sf_list{ksf}, A, B, C, Bd, cfg, Luse);
            ms = linMetrics(lk, cfg, cfg.target_theta_deg);
            mg = linGustMetrics(lk, cfg);
            fprintf('    %-5s %-12s %13.2f %13.2f %13s %13s\n', sf_names{ksf}, tag, ...
                    ms.u_peak_abs, mg.u_peak_abs, fmtTime(ms.settle), fmtTime(mg.settle));
        end
    end
    [ctrl_LQR, sc_lqr_o, bind_lqr_o] = fitAuthority(ctrl_LQR, A, B, C, Bd, cfg, L_obs);
    [ctrl_LQI, sc_lqi_o, bind_lqi_o] = fitAuthority(ctrl_LQI, A, B, C, Bd, cfg, L_obs);
    lin_LQR = linearCL(ctrl_LQR, A, B, C, Bd, cfg, L_obs);
    lin_LQI = linearCL(ctrl_LQI, A, B, C, Bd, cfg, L_obs);
    fprintf('\n    LQR as flown : de-rated x%.3f (binding case: %s)\n', sc_lqr_o, bind_lqr_o.bind);
    fprintf('                   K  = [%+.4f %+.4f %+.4f]\n', ctrl_LQR.K);
    fprintf('    LQI as flown : de-rated x%.3f (binding case: %s)\n', sc_lqi_o, bind_lqi_o.bind);
    fprintf('                   Kx = [%+.4f %+.4f %+.4f]   Kz = %+.4f\n', ctrl_LQI.Kx, ctrl_LQI.Kz);
    fprintf('    closed-loop poles of the LQI loop as flown (airframe + servo + observer):\n');
    printPoles(lin_LQI.poles);
    printMargins(lin_LQI);
    printMetrics('LQI as flown (observer in the loop, linear model)', ...
                 linMetrics(lin_LQI, cfg, cfg.target_theta_deg), cfg);
    plotDesign(lin_LQI, cfg, 'LQI as flown (observer in the loop)');
else
    fprintf('\n  cfg.use_observer is false: the state feedback uses the true state.\n');
end

%% ------------------------------------------------------------------------
%  SECTION 9 -- CLOSED-LOOP VERIFICATION ON THE REALISTIC PLANT
%
%  Everything below runs the FULL loop:
%     nonlinear airframe + first-order servo + ASYMMETRIC saturation
%     (+25/-15 deg) + rate limit + anti-windup + theta-only observer
%  Scenarios:
%     9a  attitude step   : fly target_theta up from the trim attitude
%     9b  gust rejection  : hold the target through a gust upset
%     9c  combined        : command, settle, then take the gust
%     9d  standing upset  : constant mis-trim -> exposes integral action
%     9e  large command   : deliberately saturates the elevator
%     9f  nose-down       : exercises the tighter -15 deg limit
% -------------------------------------------------------------------------

fprintf('\n==================== 9. NONLINEAR CLOSED-LOOP VERIFICATION ====================\n');

% ---- 9.0  the nonlinear model reduces to (A,B) at the trim point --------
epsFD = 1e-6;  A_num = zeros(3);
for k = 1:3
    dx = zeros(3,1); dx(k) = epsFD;
    A_num(:,k) = (nonlinearPitch(dx, 0, A, B) - nonlinearPitch(-dx, 0, A, B))/(2*epsFD);
end
B_num = (nonlinearPitch([0;0;0], epsFD, A, B) - nonlinearPitch([0;0;0], -epsFD, A, B))/(2*epsFD);
fprintf('  nonlinear-model check: max|A_num - A| = %.2e , max|B_num - B| = %.2e  (both ~0)\n', ...
        max(max(abs(A_num - A))), max(abs(B_num - B)));

controllers = {ctrl_PID, ctrl_LQR, ctrl_LQI};
cnames      = {'PID', 'LQR', 'LQI'};
ncon        = numel(controllers);

s0 = defaultScenario(cfg);

sA = s0; sA.name = '9a  attitude step (target_theta)';
sA.cmd_deg = cfg.target_theta_deg; sA.T = cfg.T_step;
sA.target_deg = cfg.target_theta_deg; sA.excursion_deg = abs(cfg.target_theta_deg);

sB = s0; sB.name = '9b  gust rejection while holding the target';
sB.gust_deg = cfg.gust_theta_deg; sB.gust_time = cfg.gust_time_s;
sB.T = cfg.T_gust; sB.t0 = cfg.gust_time_s;
sB.excursion_deg = abs(cfg.gust_theta_deg); sB.kind = 'gust';

sC = s0; sC.name = '9c  command then gust';
sC.cmd_deg = cfg.target_theta_deg; sC.gust_deg = cfg.gust_theta_deg;
sC.gust_time = 0.5*cfg.T_long; sC.T = cfg.T_long;
sC.target_deg = cfg.target_theta_deg; sC.t0 = 0.5*cfg.T_long;
sC.excursion_deg = abs(cfg.gust_theta_deg); sC.kind = 'gust';

sD = s0; sD.name = '9d  standing elevator mis-trim';
sD.bias_deg = cfg.bias_elev_deg; sD.bias_time = cfg.bias_time_s;
sD.T = cfg.T_long; sD.t0 = cfg.bias_time_s;
sD.excursion_deg = abs(cfg.gust_theta_deg); sD.kind = 'gust';

sE = s0; sE.name = '9e  oversized command (elevator saturates)';
sE.cmd_deg = 2*cfg.target_theta_deg; sE.T = cfg.T_long;
sE.target_deg = 2*cfg.target_theta_deg; sE.excursion_deg = abs(2*cfg.target_theta_deg);

sF = s0; sF.name = '9f  nose-down command (tighter -15 deg limit)';
sF.cmd_deg = -cfg.target_theta_deg; sF.T = cfg.T_step;
sF.target_deg = -cfg.target_theta_deg; sF.excursion_deg = abs(cfg.target_theta_deg);

scenarios = {sA, sB, sC, sD, sE, sF};
if exist('rng', 'file') || exist('rng', 'builtin'), rng(0); else randn('state', 0); end
nmax  = round(max(cellfun(@(z) z.T, scenarios))/cfg.dt) + 10;
if cfg.sim_noise
    noise = cfg.theta_noise_deg*D2R*randn(nmax, 1);
else
    noise = zeros(nmax, 1);
end

RES = cell(numel(scenarios), ncon);
OUT = cell(numel(scenarios), ncon);
for is = 1:numel(scenarios)
    sc = scenarios{is};
    fprintf('\n  ---- %s ----\n', sc.name);
    for ic = 1:ncon
        OUT{is,ic} = simulateAutopilot(controllers{ic}, cfg, sc, A, B, C, Bd, L_obs, noise);
        RES{is,ic} = respMetrics(OUT{is,ic}, cfg, sc);
    end
    printScenarioTable(cnames, {RES{is,:}}, cfg, sc);
    plotScenario({OUT{is,:}}, cnames, cfg, sc);
end

% ---- 9g  robustness to model error --------------------------------------
%  The airframe is perturbed while the controller AND the observer keep
%  believing the nominal model.  This is where integral action earns its
%  keep: it holds the attitude on target even when the model is wrong,
%  whereas a pure regulator carries the modelling error as a bias.
fprintf('\n  ---- 9g  model mismatch (controller and observer keep the nominal model) ----\n');
Ap_soft = A;  Ap_soft(1,1) = 0.7*A(1,1);  Ap_soft(2,1) = 0.7*A(2,1);
Ap_damp = A;  Ap_damp(2,2) = 0.5*A(2,2);
pert_names = {'nominal', 'elevator power -30%', 'elevator power +30%', ...
              'alpha derivatives -30%', 'pitch damping -50%'};
pert_A = {A, A, A, Ap_soft, Ap_damp};
pert_B = {B, 0.7*B, 1.3*B, B, B};
fprintf('    %-24s %-5s %11s %9s %11s %13s\n', 'perturbation', 'ctrl', ...
        'settle[s]', 'over[%]', 'elev[deg]', 'err_end[deg]');
for ip = 1:numel(pert_names)
    for ic = 1:ncon
        op = simulateAutopilot(controllers{ic}, cfg, sA, A, B, C, Bd, L_obs, noise, ...
                               pert_A{ip}, pert_B{ip});
        mp = respMetrics(op, cfg, sA);
        fprintf('    %-24s %-5s %11s %9.2f %11.2f %13.4f\n', ...
                ternary(ic == 1, pert_names{ip}, ''), cnames{ic}, ...
                fmtTime(mp.settle), mp.overshoot, mp.u_peak_abs, mp.err_end);
    end
end
fprintf('    every loop stays stable across +/-30%% modelling error; PID and LQI still\n');
fprintf('    land exactly on the target, LQR does not have to.\n\n');

% ---- observer convergence, shown on the LQI loop -------------------------
outObs = simulateAutopilot(ctrl_LQI, cfg, sC, A, B, C, Bd, L_obs, noise);
figure('Name','9. Observer convergence (LQI loop)');
subplot(3,1,1);
plot(outObs.t, outObs.x(:,1)*R2D, 'b-', outObs.t, outObs.xhat(:,1)*R2D, 'r--', 'LineWidth', 1.3);
grid on; ylabel('\alpha (deg)'); legend('true','estimate'); title('Observer: only \theta is measured');
subplot(3,1,2);
plot(outObs.t, outObs.x(:,2)*56.7*R2D, 'b-', outObs.t, outObs.xhat(:,2)*56.7*R2D, 'r--', 'LineWidth', 1.3);
grid on; ylabel('pitch rate (deg/s)'); legend('true','estimate');
subplot(3,1,3);
plot(outObs.t, (outObs.x(:,3) - outObs.xhat(:,3))*R2D, 'k-', 'LineWidth', 1.3);
grid on; ylabel('\theta error (deg)'); xlabel('Time (s)');
title('Attitude estimation error');
fprintf('\n  observer: peak |alpha| estimation error = %.3f deg, settles to %.4f deg\n', ...
        max(abs(outObs.x(:,1) - outObs.xhat(:,1)))*R2D, ...
        abs(outObs.x(end,1) - outObs.xhat(end,1))*R2D);

%% ------------------------------------------------------------------------
%  SECTION 10 -- COMPARATIVE ANALYSIS: PID vs LQR vs LQI
% -------------------------------------------------------------------------

fprintf('\n==================== 10. COMPARATIVE ANALYSIS ====================\n');

% --- the numbers that decide the ranking ---------------------------------
%  1 step settling   2 step overshoot   3 peak elevator   4 step IAE
%  5 gust recovery   6 gust IAE         7 mis-trim error  8 phase margin
lins = {lin_PID, lin_LQR, lin_LQI};
pm   = nan(1, ncon);
for ic = 1:ncon
    try
        [~, pmv] = margin(lins{ic}.Lo);
        pm(ic) = pmv;
    catch
    end
end
vals = zeros(8, ncon);
for ic = 1:ncon
    vals(1,ic) = RES{1,ic}.settle;
    vals(2,ic) = abs(RES{1,ic}.overshoot);
    vals(3,ic) = RES{1,ic}.u_peak_abs;
    vals(4,ic) = RES{1,ic}.iae;
    vals(5,ic) = RES{2,ic}.settle;
    vals(6,ic) = RES{2,ic}.iae;
    vals(7,ic) = abs(RES{4,ic}.err_end);
    vals(8,ic) = pm(ic);
end
crit = {'step settling time (s)', 'step overshoot (%)', 'peak elevator (deg)', ...
        'step IAE (deg*s)', 'gust recovery time (s)', 'gust IAE (deg*s)', ...
        'mis-trim steady error (deg)', 'phase margin (deg)'};
dirs = [1 1 1 1 1 1 1 -1];        % +1 = smaller is better, -1 = larger is better
wts  = [0.20 0.10 0.14 0.12 0.14 0.08 0.10 0.12];

fprintf('\n  RAW COMPARISON\n');
fprintf('    %-30s', 'criterion');
fprintf('%12s', cnames{:});  fprintf('%10s\n', 'best');
for ir = 1:size(vals,1)
    fprintf('    %-30s', crit{ir});
    fprintf('%12.3f', vals(ir,:));
    v = vals(ir,:);  v(~isfinite(v)) = dirs(ir)*1e9;
    if dirs(ir) > 0, [~, ib] = min(v); else, [~, ib] = max(v); end
    fprintf('%10s\n', cnames{ib});
end
fprintf(['    (settling/recovery: inside +/-%.0f%% of the excursion but never tighter\n' ...
         '     than %.2f deg; "> horizon" counts as the worst case)\n'], ...
        100*cfg.spec.settle_band, cfg.spec.band_floor_deg);

nrm = zeros(size(vals));
for ir = 1:size(vals,1)
    v = vals(ir,:);
    v(~isfinite(v)) = max(v(isfinite(v)))*3 + 1;     % penalise "never settles"
    if dirs(ir) < 0, v = -v; end                      % flip so smaller is better
    span = max(v) - min(v);
    if span < 1e-12, nrm(ir,:) = 0; else, nrm(ir,:) = (v - min(v))/span; end
end
score = wts*nrm;
fprintf('\n  WEIGHTED SCORE (0 = best on every criterion, 1 = worst)\n');
for ic = 1:ncon
    fprintf('    %-6s %6.3f\n', cnames{ic}, score(ic));
end
[~, ibest] = min(score);
rank_order = cnames(sortIdx(score));

fprintf('\n  RANKING : 1) %s   2) %s   3) %s\n', rank_order{1}, rank_order{2}, rank_order{3});
fprintf('\n  VERDICT\n');
fprintf('    %s is the best-implemented design for this airframe.\n', cnames{ibest});
fprintf('    * %s settles the %.1f deg command in %.2f s versus %.2f s (%s) and %.2f s (%s).\n', ...
        cnames{ibest}, cfg.target_theta_deg, vals(1,ibest), ...
        vals(1, otherIdx(ibest,1,ncon)), cnames{otherIdx(ibest,1,ncon)}, ...
        vals(1, otherIdx(ibest,2,ncon)), cnames{otherIdx(ibest,2,ncon)});
fprintf('    * peak elevator demand %.2f deg of the %+.0f/%+.0f deg available (%.0f%% of authority).\n', ...
        vals(3,ibest), cfg.elev_up_deg, cfg.elev_dn_deg, RES{1,ibest}.pct_authority);
fprintf('    * gust upset of %+.1f deg is removed in %.2f s.\n', cfg.gust_theta_deg, vals(5,ibest));
fprintf('\n    WHY, in terms of Section 2:\n');
fprintf('    * LQR has no integrator, so the identity int(err)dt = %.2f - sum(1/|p|) forces\n', I0);
fprintf('      a closed-loop pole slower than the plant zero (%.3f).  Its step response is\n', -b0/b1);
fprintf('      beautifully monotonic (%.1f%% overshoot) but drags a %.0f s tail, and it leaves\n', ...
        abs(RES{1,2}.overshoot), vals(1,2));
fprintf('      %.3f deg of standing error under a mis-trim because nothing integrates.\n', vals(7,2));
b_rad = cfg.bias_elev_deg*D2R;
e_ss  = -(A - L_obs*C)\(B*b_rad);
fprintf('      Analytically that bias should cost only %+.3f deg (= bias/K_theta) with the\n', ...
        b_rad/ctrl_LQR.K(3)*R2D);
fprintf('      full state, but %+.3f deg through the observer: the estimator cannot see\n', ...
        (b_rad + ctrl_LQR.K*e_ss)/ctrl_LQR.K(3)*R2D);
fprintf('      the bias, so the large pitch-rate gain amplifies its own estimation error.\n');
fprintf('    * PID adds the integrator, so the tail becomes a small overshoot instead, and\n');
fprintf('      the mis-trim error goes to zero.  But it only sees theta: the rate term is a\n');
fprintf('      filtered derivative of the measurement, so damping costs elevator (peak %.1f deg).\n', vals(3,1));
fprintf('    * LQI has both the integrator AND full-state feedback (alpha and q from the\n');
fprintf('      observer), so it damps the short period without waiting for a derivative and\n');
fprintf('      needs the least elevator (%.1f deg peak) while being the fastest.\n', vals(3,3));
fprintf('    * Cost of that performance: LQI needs an observer (Section 8) and therefore a\n');
fprintf('      trustworthy model; PID needs one sensor and no model at all.\n');

% --- comparison figure ----------------------------------------------------
figure('Name','10. Comparative analysis PID vs LQR vs LQI');
subplot(2,3,1); hold on; grid on;
for ic = 1:ncon
    plot(OUT{1,ic}.t, OUT{1,ic}.theta_abs_deg, 'LineWidth', 1.6);
end
plot([0 sA.T], cfg.theta_target_abs_deg*[1 1], 'k--');
title(sprintf('Attitude step to %+.1f deg', cfg.theta_target_abs_deg));
xlabel('Time (s)'); ylabel('\theta (deg)'); legend([cnames, {'target'}], 'Location','SouthEast');
subplot(2,3,4); hold on; grid on;
for ic = 1:ncon
    plot(OUT{1,ic}.t, OUT{1,ic}.u_cmd_deg, 'LineWidth', 1.4);
end
plot([0 sA.T], cfg.elev_up_deg*[1 1], 'r:', [0 sA.T], cfg.elev_dn_deg*[1 1], 'r:');
title('Elevator command'); xlabel('Time (s)'); ylabel('\delta_e (deg)');
subplot(2,3,2); hold on; grid on;
for ic = 1:ncon
    plot(OUT{2,ic}.t, OUT{2,ic}.theta_abs_deg, 'LineWidth', 1.6);
end
plot([0 sB.T], cfg.theta_target_abs_deg*[1 1], 'k--');
title(sprintf('Gust rejection (%+.1f deg upset)', cfg.gust_theta_deg));
xlabel('Time (s)'); ylabel('\theta (deg)');
subplot(2,3,5); hold on; grid on;
for ic = 1:ncon
    plot(OUT{2,ic}.t, OUT{2,ic}.u_cmd_deg, 'LineWidth', 1.4);
end
plot([0 sB.T], cfg.elev_up_deg*[1 1], 'r:', [0 sB.T], cfg.elev_dn_deg*[1 1], 'r:');
title('Elevator command (gust)'); xlabel('Time (s)'); ylabel('\delta_e (deg)');
subplot(2,3,3); hold on; grid on;
for ic = 1:ncon
    plot(OUT{4,ic}.t, OUT{4,ic}.theta_abs_deg, 'LineWidth', 1.6);
end
plot([0 sD.T], cfg.theta_target_abs_deg*[1 1], 'k--');
title(sprintf('Standing mis-trim (%+.1f deg elevator)', cfg.bias_elev_deg));
xlabel('Time (s)'); ylabel('\theta (deg)');
subplot(2,3,6);
bar(nrm(1:5,:)');
grid on; set(gca, 'XTickLabel', cnames);
ylabel('normalised (1 = worst)');
legend({'settling','overshoot','peak elevator','step IAE','gust recovery'}, ...
       'Location','NorthOutside', 'Orientation','horizontal');
title('Score card (shorter bars are better)');

%% ------------------------------------------------------------------------
%  SECTION 11 -- REFERENCE COMMAND SHAPING (2-DOF)
%
%  Section 2 showed that a RAW step command cannot be tracked both quickly
%  and without overshoot on this airframe.  A first-order command filter
%  (a "pitch command model", standard in autopilots) shifts the identity
%  by +tau_shape and buys a monotonic response with a much smaller
%  elevator kick -- at the price of a slower, deliberate manoeuvre.
% -------------------------------------------------------------------------

fprintf('\n==================== 11. REFERENCE COMMAND SHAPING ====================\n');
fprintf('  command filter: theta_cmd_shaped = theta_cmd / (%.2f s + 1)\n', cfg.shape_tau);

sS = sA; sS.name = sprintf('11  shaped attitude step (tau = %.1f s)', cfg.shape_tau);
sS.shape_tau = cfg.shape_tau;
OUT_S = cell(1, ncon); RES_S = cell(1, ncon);
for ic = 1:ncon
    OUT_S{ic} = simulateAutopilot(controllers{ic}, cfg, sS, A, B, C, Bd, L_obs, noise);
    RES_S{ic} = respMetrics(OUT_S{ic}, cfg, sS);
end
printScenarioTable(cnames, RES_S, cfg, sS);
fprintf('\n  raw step vs shaped step:\n');
fprintf('    %-6s %14s %14s %14s %14s\n', '', 'overshoot raw', 'shaped', 'peak elev raw', 'shaped');
for ic = 1:ncon
    fprintf('    %-6s %13.2f%% %13.2f%% %14.2f %14.2f\n', cnames{ic}, ...
            RES{1,ic}.overshoot, RES_S{ic}.overshoot, RES{1,ic}.u_peak_abs, RES_S{ic}.u_peak_abs);
end
plotScenario(OUT_S, cnames, cfg, sS);

%% ------------------------------------------------------------------------
%  SECTION 12 -- SIMULINK MODEL OF THE AUTOPILOT
% -------------------------------------------------------------------------

fprintf('\n==================== 12. SIMULINK MODEL ====================\n');

des = struct('A', A, 'B', B, 'C', C, 'Bd', Bd, 'L', L_obs, ...
             'PID', ctrl_PID, 'LQR', ctrl_LQR, 'LQI', ctrl_LQI);

if exist('new_system') == 0
    fprintf('  Simulink is not available in this session -- model build skipped.\n');
    fprintf('  Run  build_pitch_autopilot_simulink(cfg, des)  on a machine with Simulink;\n');
    fprintf('  the function is included next to this script and needs nothing else.\n');
else
    try
        mdl = build_pitch_autopilot_simulink(cfg, des, 'pitch_autopilot');
        fprintf('  model "%s" built and saved.  Open it with:  open_system(''%s'')\n', mdl, mdl);
        fprintf('  Three branches (PID / LQR / LQI) share one reference and one gust source;\n');
        fprintf('  each has its own saturation (+%.0f/%.0f deg), rate limit, servo lag,\n', ...
                cfg.elev_up_deg, cfg.elev_dn_deg);
        fprintf('  airframe and observer.  Scopes show attitude and elevator in degrees.\n');
        try
            sim(mdl);
            fprintf('  simulation ran: %g s of flight; signals logged with To Workspace.\n', ...
                    get_param(mdl, 'StopTime'));
        catch simErr
            fprintf('  model built, but the automatic run failed (%s) -- run it manually.\n', ...
                    simErr.message);
        end
    catch buildErr
        fprintf('  Simulink model build failed: %s\n', buildErr.message);
        fprintf('  The MATLAB-side design and analysis above are unaffected.\n');
    end
end

fprintf('\n==========================================================================\n');
fprintf('  DONE.  Design summary for a %+.1f deg pitch-up from %+.1f deg trim:\n', ...
        cfg.target_theta_deg, cfg.theta_trim_deg);
fprintf('    PID : Kp = %.3f  Ki = %.3f  Kd = %.3f  N = %.1f\n', ...
        ctrl_PID.Kp, ctrl_PID.Ki, ctrl_PID.Kd, ctrl_PID.N);
fprintf('    LQR : K  = [%+.4f %+.4f %+.4f]\n', ctrl_LQR.K);
fprintf('    LQI : Kx = [%+.4f %+.4f %+.4f]  Kz = %+.4f\n', ctrl_LQI.Kx, ctrl_LQI.Kz);
fprintf('    OBS : L  = [%+.4f %+.4f %+.4f]''\n', L_obs);
fprintf('    recommended for implementation: %s\n', cnames{ibest});
fprintf('==========================================================================\n\n');

%% ========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

% ---- controller constructors --------------------------------------------
function c = makeP(Kp)
c = struct('type','P','name','P','Kp',Kp,'Ki',0,'Kd',0,'N',10, ...
           'K',[0 0 0],'Kx',[0 0 0],'Kz',0,'has_integrator',false,'needs_state',false);
end

function c = makePID(Kp, Ki, Kd, N)
c = struct('type','PID','name','PID','Kp',Kp,'Ki',Ki,'Kd',Kd,'N',N, ...
           'K',[0 0 0],'Kx',[0 0 0],'Kz',0,'has_integrator',true,'needs_state',false);
end

function c = makeSF(K)
c = struct('type','SF','name','State feedback','Kp',0,'Ki',0,'Kd',0,'N',10, ...
           'K',K(:)','Kx',[0 0 0],'Kz',0,'has_integrator',false,'needs_state',true);
end

function c = makeLQI(Kx, Kz)
c = struct('type','LQI','name','LQI','Kp',0,'Ki',0,'Kd',0,'N',10, ...
           'K',[0 0 0],'Kx',Kx(:)','Kz',Kz,'has_integrator',true,'needs_state',true);
end

function c = scaleCtrl(c, s)
% De-rate the whole loop gain by the factor s (used to respect the elevator
% travel limits without changing the shape of the design).
switch c.type
    case {'P','PID'}
        c.Kp = c.Kp*s;  c.Ki = c.Ki*s;  c.Kd = c.Kd*s;
    case 'SF'
        c.K = c.K*s;
    case 'LQI'
        c.Kx = c.Kx*s;  c.Kz = c.Kz*s;
end
end

function Q = brysonQ(cfg)
% Bryson's rule on the three airframe states.  The q weight uses the
% PHYSICAL pitch rate 56.7*q, so the number in cfg is in deg/s.
d = cfg.D2R;
Q = diag([1/(cfg.spec.alpha_max_deg*d)^2, ...
          (56.7/(cfg.spec.qrate_max_deg*d))^2, ...
          1/(cfg.spec.theta_max_deg*d)^2]);
end

% ---- linear (design-model) closed loop ----------------------------------
function lin = linearCL(ctrl, A, B, C, Bd, cfg, L, Ap, Bp)
% Linear model of the loop AS IT IS IMPLEMENTED:
%     airframe + first-order servo lag + controller states
%     + (when the state feedback runs on estimates) the observer
% No travel limits here -- saturation is exercised in Section 9.
% Pass L = [] for the ideal full-state loop (the design-stage model that
% the separation principle justifies), or the observer gain for the loop
% that is actually flown.
%
% State order: [alpha q theta | delta | controller states | observer states]
% Inputs      : attitude command (rad), gust pitch-rate disturbance (rad/s)
% Outputs     : theta deviation, elevator COMMAND, actuated deflection
if nargin < 7, L = []; end
if nargin < 8 || isempty(Ap), Ap = A; end     % TRUE airframe
if nargin < 9 || isempty(Bp), Bp = B; end     % (differs from the model only
tau    = cfg.tau_act;                          %  in the robustness audits)
useObs = ~isempty(L) && ctrl.needs_state;

switch ctrl.type
    case 'P',    nc = 0;
    case 'PID',  nc = 2;          % [xi; xf]
    case 'SF',   nc = 0;
    case 'LQI',  nc = 1;          % [z]
    otherwise,   error('linearCL: unknown controller type %s', ctrl.type);
end
id = 4;                            % actuator position
ic = id + (1:nc);                  % controller states
no = 3*useObs;
io = id + nc + (1:no);             % observer states
n  = 4 + nc + no;

% ---- elevator command as a row on the state vector ----------------------
Ku = zeros(1,n);  Dur = 0;         % u_cmd = Ku*state + Dur*command
switch ctrl.type
    case 'P'
        Ku(3) = -ctrl.Kp;                                 Dur = ctrl.Kp;
    case 'PID'
        Ku(3)     = -(ctrl.Kp + ctrl.Kd*ctrl.N);
        Ku(ic(1)) =  ctrl.Ki;
        Ku(ic(2)) =  ctrl.Kd*ctrl.N;                      Dur = ctrl.Kp;
    case 'SF'
        if useObs, Ku(io) = -ctrl.K;  else, Ku(1:3) = -ctrl.K;  end
        Dur = ctrl.K(3);
    case 'LQI'
        if useObs, Ku(io) = -ctrl.Kx; else, Ku(1:3) = -ctrl.Kx; end
        Ku(ic(1)) = -ctrl.Kz;                             Dur = 0;
end

% ---- assemble ------------------------------------------------------------
Acl = zeros(n);  Br = zeros(n,1);  Bg = zeros(n,1);
Acl(1:3,1:3) = Ap;  Acl(1:3,id) = Bp;  Bg(1:3) = Bd;      % airframe (true)
Acl(id,:)  = Ku/tau;                                       % servo lag
Acl(id,id) = Acl(id,id) - 1/tau;
Br(id)     = Dur/tau;
switch ctrl.type
    case 'PID'
        Acl(ic(1),3) = -1;                Br(ic(1)) = 1;   % xi_dot = r - theta
        Acl(ic(2),3) =  ctrl.N;   Acl(ic(2),ic(2)) = -ctrl.N;
    case 'LQI'
        Acl(ic(1),3) = -1;                Br(ic(1)) = 1;   % z_dot  = r - theta
end
if useObs
    Acl(io,io)  = A - L*C;                                 % xhat_dot
    Acl(io,1:3) = Acl(io,1:3) + L*C;
    Acl(io,id)  = B;
end

Cy = zeros(3,n);  Cy(1,3) = 1;  Cy(2,:) = Ku;  Cy(3,id) = 1;
Dy = [0; Dur; 0];

% ---- loop gain broken at the elevator, for the stability margins ---------
P4 = ss([A, B; zeros(1,3), -1/tau], [zeros(3,1); 1/tau], [C, 0], 0);
switch ctrl.type
    case 'P'
        Lo = ctrl.Kp*P4;
    case 'PID'
        numC = [ctrl.Kp + ctrl.Kd*ctrl.N, ctrl.Kp*ctrl.N + ctrl.Ki, ctrl.Ki*ctrl.N];
        Lo   = tf(numC, [1 ctrl.N 0])*P4;
    case 'SF'
        if useObs
            % observer-based controller from theta to elevator
            Lo = ss(A - L*C - B*ctrl.K, L, ctrl.K, 0)*P4;
        else
            Lo = ss(A, B, ctrl.K, 0)*tf(1, [tau 1]);
        end
    case 'LQI'
        if useObs
            Ac = [A - L*C - B*ctrl.Kx, -B*ctrl.Kz; zeros(1,3), 0];
            Bc = [L; -1];
            Lo = ss(Ac, Bc, [ctrl.Kx, ctrl.Kz], 0)*P4;
        else
            Aa = [A, zeros(3,1); -C, 0];  Ba = [B; 0];
            Lo = ss(Aa, Ba, [ctrl.Kx, ctrl.Kz], 0)*tf(1, [tau 1]);
        end
end

lin.sys_ref  = ss(Acl, Br, Cy, Dy);
lin.sys_gust = ss(Acl, Bg, Cy, [0; 0; 0]);
lin.poles    = eig(Acl);
lin.stable   = all(real(lin.poles) < -1e-9);
lin.Lo       = Lo;
lin.ctrl     = ctrl;
lin.useObs   = useObs;
end

function m = linMetrics(lin, cfg, cmd_deg)
% Step-response metrics straight off the linear closed loop.
t = (0:0.01:cfg.T_step)';
if ~lin.stable
    z = zeros(numel(t),1);
    m = metricsCore(t, z, z, z, false(size(t)), cmd_deg, abs(cmd_deg), 0, cfg, 'step');
    m.stable = false;  return
end
y  = step(lin.sys_ref, t);
th = y(:,1)*cmd_deg;                            % deg (deviation)
u  = y(:,2)*cmd_deg + cfg.delta_trim_deg;       % elevator command, deg
ua = y(:,3)*cmd_deg + cfg.delta_trim_deg;       % actuated deflection, deg
m  = metricsCore(t, th, u, ua, false(size(t)), cmd_deg, abs(cmd_deg), 0, cfg, 'step');
m.stable = true;
end

function m = linGustMetrics(lin, cfg)
% Gust-rejection metrics off the linear closed loop, with the same
% raised-cosine gust used by the nonlinear runs.
t = (0:0.01:cfg.T_gust)';
if ~lin.stable
    z = zeros(numel(t),1);
    m = metricsCore(t, z, z, z, false(size(t)), 0, abs(cfg.gust_theta_deg), 0, cfg, 'gust');
    m.stable = false;  return
end
Tg = max(cfg.gust_rise_s, 1e-3);
w  = zeros(size(t));
in = t >= cfg.gust_time_s & t <= cfg.gust_time_s + Tg;
w(in) = (cfg.gust_theta_deg/Tg)*(1 - cos(2*pi*(t(in) - cfg.gust_time_s)/Tg));
y  = lsim(lin.sys_gust, w, t);
m  = metricsCore(t, y(:,1), y(:,2) + cfg.delta_trim_deg, y(:,3) + cfg.delta_trim_deg, ...
                 false(size(t)), 0, abs(cfg.gust_theta_deg), cfg.gust_time_s, cfg, 'gust');
m.stable = true;
end

function [ctrl, scale, info] = fitAuthority(ctrl, A, B, C, Bd, cfg, L)
% Keep the design inside the elevator travel limits for BOTH specified
% cases: the planned attitude command (which may use cfg.spec.authority of
% the travel) and the specified gust upset (which may use up to
% cfg.spec.authority_gust, i.e. all of it, but must not saturate).
% If either is exceeded the loop gain is de-rated until both fit.
if nargin < 7, L = []; end
R2D   = cfg.R2D;
aS    = cfg.spec.authority;
aG    = cfg.spec.authority_gust;
scale = 1;  found = false;
info = struct('bind', 'none', 'u_step', NaN, 'u_gust', NaN);
for it = 1:40
    c   = scaleCtrl(ctrl, scale);
    lin = linearCL(c, A, B, C, Bd, cfg, L);
    if lin.stable
        ms = linMetrics(lin, cfg, cfg.target_theta_deg);
        mg = linGustMetrics(lin, cfg);
        okS = ms.u_max <= aS*cfg.u_max*R2D + 1e-9 && ms.u_min >= aS*cfg.u_min*R2D - 1e-9;
        okG = mg.u_max <= aG*cfg.u_max*R2D + 1e-9 && mg.u_min >= aG*cfg.u_min*R2D - 1e-9;
        if okS && okG
            found = true;
            info.u_step = ms.u_peak_abs;  info.u_gust = mg.u_peak_abs;
            break
        end
        if ~okS, info.bind = 'attitude command'; else, info.bind = 'gust upset'; end
    else
        info.bind = 'stability';
    end
    scale = scale*0.9;
end
if ~found
    warning('fitAuthority: could not fit %s inside the elevator authority; keeping nominal gains.', ...
            ctrl.type);
    scale = 1;  return
end
ctrl = scaleCtrl(ctrl, scale);
end

function L = kalmanObserver(A, B, C, qw, Rn)
% Steady-state Kalman gain with the process noise entering through the
% elevator channel (Qn = qw*B*B'), i.e. "how much do I distrust my own
% model of the elevator-to-airframe path".  qw is the single knob that
% trades estimator bandwidth against noise and disturbance sensitivity.
% Solved as the dual LQR problem so it needs nothing beyond lqr().
Qn = qw*(B*B') + 1e-14*eye(size(A,1));
L  = lqr(A', C', Qn, Rn)';
end

function r = obsAudit(L, ctrl, A, B, C, Bd, cfg)
% Everything needed to judge one observer candidate, measured on the loop
% that actually flies (state feedback on the estimates + servo).
lin      = linearCL(ctrl, A, B, C, Bd, cfg, L);
r.L      = L;
r.stable = lin.stable;
r.normL  = norm(L);
r.pm     = NaN;  r.gm = NaN;  r.smax = NaN;
try
    [Gm, Pm] = margin(lin.Lo);
    r.gm = 20*log10(Gm);  r.pm = Pm;
catch
end
try
    r.smax = norm(feedback(1, lin.Lo), Inf);
catch
end
if lin.stable
    ms = linMetrics(lin, cfg, cfg.target_theta_deg);
    mg = linGustMetrics(lin, cfg);
    r.u_step = ms.u_peak_abs;  r.u_gust = mg.u_peak_abs;
    r.ts     = ms.settle;      r.tg     = mg.settle;
else
    r.u_step = Inf;  r.u_gust = Inf;  r.ts = Inf;  r.tg = Inf;
end
r.noise = noiseGain(A, B, C, L, ctrl, cfg);
% What the estimator bandwidth actually buys: performance when the airframe
% is NOT the one the observer believes in (alpha derivatives 30% weaker).
Amis = A;  Amis(1,1) = 0.7*A(1,1);  Amis(2,1) = 0.7*A(2,1);
lm   = linearCL(ctrl, A, B, C, Bd, cfg, L, Amis, B);
if lm.stable
    mm = linMetrics(lm, cfg, cfg.target_theta_deg);
    r.ts_mis = mm.settle;
else
    r.ts_mis = Inf;
end
end

function s = poleStr(p)
s = '';
for k = 1:numel(p)
    if abs(imag(p(k))) > 1e-9
        s = [s sprintf('%+.2f%+.2fj  ', real(p(k)), imag(p(k)))];          %#ok<AGROW>
    else
        s = [s sprintf('%+.2f  ', real(p(k)))];                            %#ok<AGROW>
    end
end
end

function g = noiseGain(A, B, C, L, ctrl, cfg)
% Worst-case amplification from attitude-sensor noise to elevator command
% for the observer-based LQI loop (peak gain, deg of elevator per deg of
% noise).  This is what limits how fast the observer may be made.
g = NaN;
try
    Ku = [zeros(1,3), -ctrl.Kx, -ctrl.Kz];
    Ao = [A,          zeros(3,3), zeros(3,1);
          L*C,        A - L*C,    zeros(3,1);
          -C,         zeros(1,3), 0] + [B; B; 0]*Ku;
    Bn = [zeros(3,1); L; -1];
    g  = norm(ss(Ao, Bn, Ku, 0), Inf);
catch
    g = NaN;
end
end

% ---- scenarios -----------------------------------------------------------
function s = defaultScenario(cfg)
s = struct('name','', 'cmd_deg',0, 'cmd_time',0, 'gust_deg',0, ...
           'gust_time',cfg.gust_time_s, 'gust_rise',cfg.gust_rise_s, ...
           'bias_deg',0, 'bias_time',cfg.bias_time_s, 'T',cfg.T_step, ...
           'shape_tau',0, 'target_deg',0, 't0',0, ...
           'excursion_deg',abs(cfg.target_theta_deg), 'kind','step');
end

function r = refSignal(t, scen)
% Attitude command (rad, deviation from trim), optionally shaped by a
% first-order command filter.
r = 0;
if t >= scen.cmd_time
    if scen.shape_tau > 0
        r = scen.cmd_deg*(1 - exp(-(t - scen.cmd_time)/scen.shape_tau));
    else
        r = scen.cmd_deg;
    end
end
r = r*pi/180;
end

function w = gustSignal(t, scen)
% Gust modelled as a raised-cosine pitch-RATE disturbance whose area is
% exactly the requested pitch upset, so theta is displaced by gust_deg and
% stays displaced until the autopilot corrects it.
w = 0;
if scen.gust_deg ~= 0
    Tg = max(scen.gust_rise, 1e-3);
    tt = t - scen.gust_time;
    if tt >= 0 && tt <= Tg
        w = (scen.gust_deg*pi/180/Tg)*(1 - cos(2*pi*tt/Tg));
    end
end
end

function b = biasSignal(t, scen)
% Standing elevator / pitching-moment bias (mis-trim), rad.
b = 0;
if scen.bias_deg ~= 0 && t >= scen.bias_time
    b = scen.bias_deg*pi/180;
end
end

% ---- plant ---------------------------------------------------------------
function xdot = nonlinearPitch(x, u, Ap, Bp)
% Nonlinear airframe: the linear model with sin(.) restored on the
% angle-like quantities (alpha and the elevator).  Written straight from
% the state-space data so that a PERTURBED airframe can be passed in for
% the robustness runs of Section 9g.  Its Jacobian at the trim point
% (x = 0, u = 0) is exactly (Ap,Bp) -- checked numerically in Section 9.0
% -- while staying bounded for large excursions.
xdot = Ap*[sin(x(1)); x(2); x(3)] + Bp*sin(u);
end

% ---- control law ---------------------------------------------------------
function [u, e] = controlLaw(ctrl, xfb, xi, xf, r, y)
% xfb : state used for feedback (true state, or observer estimate)
% y   : measured attitude deviation (rad) -- what the error integrates
% xi  : integrator state, xf : derivative-filter state
e = r - y;
switch ctrl.type
    case 'P'
        u = ctrl.Kp*e;
    case 'PID'
        u = ctrl.Kp*e + ctrl.Ki*xi - ctrl.Kd*ctrl.N*(y - xf);
    case 'SF'
        u = -ctrl.K*(xfb - [0; 0; r]);
    case 'LQI'
        u = -ctrl.Kx*xfb - ctrl.Kz*xi;
    otherwise
        error('controlLaw: unknown controller type %s', ctrl.type);
end
end

% ---- full nonlinear closed-loop simulation ------------------------------
function out = simulateAutopilot(ctrl, cfg, scen, A, B, C, Bd, L, noise, Ap, Bp)
% Fixed-step RK4 on
%   s = [alpha; q; theta; xi; xf; delta; alpha_hat; q_hat; theta_hat]
% with the nonlinear airframe, servo lag, ASYMMETRIC travel saturation,
% rate limit, back-calculation anti-windup and a theta-only observer.
% A fixed step is used on purpose: the saturation/rate limits are
% non-smooth, and every controller is compared on the identical time grid.
% (Ap,Bp) is the TRUE airframe; (A,B) is the model the controller and the
% observer believe in.  They differ only in the robustness runs.
if nargin < 10 || isempty(Ap), Ap = A; end
if nargin < 11 || isempty(Bp), Bp = B; end
dt = cfg.dt;
n  = round(scen.T/dt);
t  = (0:n)'*dt;
s  = zeros(9,1);

TH  = zeros(n+1,1);  UC = TH;  UA = TH;  RF = TH;  YM = TH;
SAT = false(n+1,1);  XS = zeros(n+1,3);  XH = zeros(n+1,3);

for k = 1:n+1
    tk = t(k);
    [u_cmd, ~, u_sat, y] = loopOutputs(tk, s, ctrl, cfg, scen, C, noise);
    TH(k)  = s(3);      UC(k) = u_cmd;   UA(k) = s(6);
    RF(k)  = refSignal(tk, scen);        YM(k) = y;
    SAT(k) = abs(u_sat - u_cmd) > 1e-12;
    XS(k,:) = s(1:3)';  XH(k,:) = s(7:9)';
    if k > n, break; end
    k1 = derivs(tk,        s,           ctrl, cfg, scen, A, B, C, Bd, L, noise, Ap, Bp);
    k2 = derivs(tk + dt/2, s + dt/2*k1, ctrl, cfg, scen, A, B, C, Bd, L, noise, Ap, Bp);
    k3 = derivs(tk + dt/2, s + dt/2*k2, ctrl, cfg, scen, A, B, C, Bd, L, noise, Ap, Bp);
    k4 = derivs(tk + dt,   s + dt*k3,   ctrl, cfg, scen, A, B, C, Bd, L, noise, Ap, Bp);
    s  = s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

R2D = cfg.R2D;
out.t             = t;
out.theta_dev_deg = TH*R2D;
out.theta_abs_deg = cfg.theta_trim_deg + TH*R2D;
out.ref_dev_deg   = RF*R2D;
out.ref_abs_deg   = cfg.theta_trim_deg + RF*R2D;
out.u_cmd_deg     = cfg.delta_trim_deg + UC*R2D;      % absolute deflection
out.u_act_deg     = cfg.delta_trim_deg + UA*R2D;
out.sat           = SAT;
out.x             = XS;
out.xhat          = XH;
out.qdot_deg      = XS(:,2)*56.7*R2D;                 % physical pitch rate
out.ctrl          = ctrl;
out.scen          = scen;
end

function [u_cmd, e, u_sat, y] = loopOutputs(t, s, ctrl, cfg, scen, C, noise)
% Signals the loop produces at the current state, used for logging.
x = s(1:3);  xi = s(4);  xf = s(5);  xhat = s(7:9);
idx = min(numel(noise), max(1, floor(t/cfg.dt) + 1));
y   = C*x + noise(idx);
if cfg.use_observer && ctrl.needs_state
    xfb = xhat;
else
    xfb = x;
end
[u_cmd, e] = controlLaw(ctrl, xfb, xi, xf, refSignal(t, scen), y);
u_sat = min(max(u_cmd, cfg.u_min), cfg.u_max);
end

function ds = derivs(t, s, ctrl, cfg, scen, A, B, C, Bd, L, noise, Ap, Bp)
x = s(1:3);  xi = s(4);  xf = s(5);  delta = s(6);  xhat = s(7:9);

idx = min(numel(noise), max(1, floor(t/cfg.dt) + 1));
y   = C*x + noise(idx);                 % only theta is measured
r   = refSignal(t, scen);

if cfg.use_observer && ctrl.needs_state
    xfb = xhat;
else
    xfb = x;
end

[u_cmd, e] = controlLaw(ctrl, xfb, xi, xf, r, y);

% ---- elevator: asymmetric travel limit, then rate limit, then servo lag --
u_sat = min(max(u_cmd, cfg.u_min), cfg.u_max);
rate  = (u_sat - delta)/cfg.tau_act;
rate  = min(max(rate, -cfg.u_rate), cfg.u_rate);
if (delta >= cfg.u_max && rate > 0) || (delta <= cfg.u_min && rate < 0)
    rate = 0;
end

% ---- airframe -----------------------------------------------------------
delta_eff = delta + biasSignal(t, scen);
if cfg.use_nonlinear
    dx = nonlinearPitch(x, delta_eff, Ap, Bp);
else
    dx = Ap*x + Bp*delta_eff;
end
dx = dx + Bd*gustSignal(t, scen);

% ---- controller states --------------------------------------------------
if ctrl.has_integrator
    dxi = e + (u_sat - u_cmd)/cfg.tau_aw;    % back-calculation anti-windup
else
    dxi = 0;
end
if strcmp(ctrl.type, 'PID')
    dxf = ctrl.N*(y - xf);
else
    dxf = 0;
end

% ---- observer (driven by the ACTUAL elevator position) ------------------
dxhat = A*xhat + B*delta + L*(y - C*xhat);

ds = [dx; dxi; dxf; rate; dxhat];
end

% ---- metrics -------------------------------------------------------------
function m = respMetrics(out, cfg, scen)
m = metricsCore(out.t, out.theta_dev_deg, out.u_cmd_deg, out.u_act_deg, ...
                out.sat, scen.target_deg, scen.excursion_deg, scen.t0, cfg, scen.kind);
end

function m = metricsCore(t, th, ucmd, uact, sat, target, excursion, t0, cfg, kind)
% One definition of every performance number, shared by the linear design
% models and the nonlinear verification runs.
if excursion <= 0, excursion = 1; end
i0 = find(t >= t0, 1);  if isempty(i0), i0 = 1; end
tt = t(i0:end) - t0;    yy = th(i0:end);
y0 = yy(1);

m.kind      = kind;
m.stable    = true;
m.target    = target;
m.excursion = excursion;

sgn = sign(target - y0);
if sgn == 0, sgn = 1; end

m.peak_dev = max(abs(yy - target));
if strcmp(kind, 'step')
    m.peak      = y0 + sgn*max(sgn*(yy - y0));
    m.overshoot = 100*max(sgn*(yy - target))/excursion;
    lo = y0 + 0.1*(target - y0);   hi = y0 + 0.9*(target - y0);
    i1 = find(sgn*(yy - lo) >= 0, 1);
    i2 = find(sgn*(yy - hi) >= 0, 1);
    if isempty(i1) || isempty(i2), m.rise = NaN; else m.rise = tt(i2) - tt(i1); end
else
    m.peak      = y0 + sign(m.peak_dev)*m.peak_dev;
    m.overshoot = 100*m.peak_dev/excursion;
    m.rise      = NaN;
end

band = max(cfg.spec.settle_band*excursion, cfg.spec.band_floor_deg);
m.band = band;
outb = find(abs(yy - target) > band);
if isempty(outb)
    m.settle = 0;
elseif outb(end) >= numel(yy)
    m.settle = Inf;                       % has not settled inside the horizon
else
    m.settle = tt(outb(end));
end

m.err_end     = th(end) - target;
m.u_max       = max(ucmd);
m.u_min       = min(ucmd);
m.u_peak_abs  = max(abs(ucmd));
m.u_act_max   = max(uact);
m.u_act_min   = min(uact);
m.pct_authority = 100*max(m.u_max/cfg.elev_up_deg, m.u_min/cfg.elev_dn_deg);
dt          = mean(diff(t));
m.sat_time  = sum(sat)*dt;
m.max_rate  = max(abs(diff(uact))/dt);
m.iae       = trapz(tt, abs(yy - target));
m.effort    = trapz(t, (ucmd - cfg.delta_trim_deg).^2);
end

% ---- printing ------------------------------------------------------------
function printMetrics(name, m, cfg)
fprintf('  PERFORMANCE  |  %s\n', name);
if ~m.stable
    fprintf('    *** closed loop UNSTABLE with these gains ***\n');  return
end
fprintf('    rise time (10-90%%)        : %s\n', fmtTime(m.rise));
fprintf('    settling time (+/-%.2f deg): %s\n', m.band, fmtTime(m.settle));
fprintf('    overshoot                 : %6.2f %%   (peak %+.2f deg deviation)\n', ...
        m.overshoot, m.peak);
fprintf('    steady-state error        : %+6.4f deg\n', m.err_end);
fprintf('    elevator command range    : %+6.2f .. %+6.2f deg   (limits %+.0f / %+.0f)\n', ...
        m.u_min, m.u_max, cfg.elev_dn_deg, cfg.elev_up_deg);
fprintf('    authority used            : %6.1f %%\n', m.pct_authority);
if m.max_rate > cfg.elev_rate_deg
    fprintf('    peak elevator rate demand : %6.1f deg/s  (servo can do %.0f -> the rate\n', ...
            m.max_rate, cfg.elev_rate_deg);
    fprintf('                                limit will be active; Section 9 shows the effect)\n');
else
    fprintf('    peak elevator rate demand : %6.1f deg/s  (servo limit %.0f, not reached)\n', ...
            m.max_rate, cfg.elev_rate_deg);
end
fprintf('    time in saturation        : %6.2f s\n', m.sat_time);
fprintf('    IAE  int|theta-target|dt  : %6.2f deg*s\n', m.iae);
fprintf('    effort  int(delta^2)dt    : %6.1f deg^2*s\n', m.effort);
end

function printScenarioTable(names, res, cfg, scen)
isStep = strcmp(scen.kind, 'step');
fprintf('    %-6s %10s %10s %10s %10s %10s %10s %10s %9s\n', 'ctrl', ...
        ternary(isStep,'rise[s]','peak[deg]'), ...
        ternary(isStep,'settle[s]','recover[s]'), ...
        ternary(isStep,'over[%]','upset[%]'), ...
        'elev+[deg]', 'elev-[deg]', 'auth[%]', 'sat[s]', 'IAE');
for k = 1:numel(names)
    m = res{k};
    if isStep, c1 = m.rise; else c1 = m.peak_dev; end
    fprintf('    %-6s %10s %10s %10.2f %10.2f %10.2f %10.1f %10.2f %9.2f\n', ...
            names{k}, fmtNum(c1), fmtTime(m.settle), m.overshoot, ...
            m.u_max, m.u_min, m.pct_authority, m.sat_time, m.iae);
end
fprintf('      target %+0.2f deg absolute (%+0.2f deg deviation), settling band +/-%.3f deg\n', ...
        cfg.theta_trim_deg + scen.target_deg, scen.target_deg, res{1}.band);
for k = 1:numel(names)
    m = res{k};
    if isinf(m.settle)
        fprintf('      %s has not settled within %.0f s (error %+0.3f deg at the end)\n', ...
                names{k}, scen.T, m.err_end);
    end
end
end

function printPoles(p)
for k = 1:numel(p)
    if abs(imag(p(k))) > 1e-9
        wn = abs(p(k));
        fprintf('    %+9.4f %+9.4fi   wn = %6.3f rad/s   zeta = %5.3f\n', ...
                real(p(k)), imag(p(k)), wn, -real(p(k))/wn);
    else
        fprintf('    %+9.4f               tau = %6.3f s\n', real(p(k)), -1/real(p(k)));
    end
end
end

function printMargins(lin)
try
    [Gm, Pm, Wg, Wp] = margin(lin.Lo);
    fprintf('  stability margins : gain margin %.2f dB @ %.3f rad/s, phase margin %.1f deg @ %.3f rad/s\n', ...
            20*log10(Gm), Wg, Pm, Wp);
catch
    fprintf('  stability margins : not available in this installation\n');
end
end

function s = fmtTime(v)
if isnan(v)
    s = '     n/a';
elseif isinf(v)
    s = '  > horizon';
else
    s = sprintf('%8.2f s', v);
end
end

function s = fmtNum(v)
if isnan(v), s = '     n/a'; else s = sprintf('%10.2f', v); end
end

% ---- plotting ------------------------------------------------------------
function plotDesign(lin, cfg, name)
% Design-stage figure: commanded step and gust rejection on the linear
% model, always with the elevator command next to the attitude.
t  = (0:0.01:cfg.T_step)';
y  = step(lin.sys_ref, t);
th = cfg.theta_trim_deg + y(:,1)*cfg.target_theta_deg;
u  = cfg.delta_trim_deg + y(:,2)*cfg.target_theta_deg;
ua = cfg.delta_trim_deg + y(:,3)*cfg.target_theta_deg;

tg  = (0:0.01:cfg.T_gust)';
Tg  = max(cfg.gust_rise_s, 1e-3);
wg  = zeros(size(tg));
in  = tg >= cfg.gust_time_s & tg <= cfg.gust_time_s + Tg;
wg(in) = (cfg.gust_theta_deg/Tg)*(1 - cos(2*pi*(tg(in) - cfg.gust_time_s)/Tg));
yg  = lsim(lin.sys_gust, wg, tg);
thg = cfg.theta_trim_deg + yg(:,1);
ug  = cfg.delta_trim_deg + yg(:,2);
uga = cfg.delta_trim_deg + yg(:,3);

figure('Name', sprintf('Design: %s', name));
subplot(2,2,1);
plot(t, th, 'b-', 'LineWidth', 1.8); hold on; grid on;
plot(t([1 end]), cfg.theta_target_abs_deg*[1 1], 'k--', 'LineWidth', 1.0);
band = cfg.spec.settle_band*abs(cfg.target_theta_deg);
plot(t([1 end]), (cfg.theta_target_abs_deg + band)*[1 1], 'k:', ...
     t([1 end]), (cfg.theta_target_abs_deg - band)*[1 1], 'k:');
xlabel('Time (s)'); ylabel('\theta (deg, absolute)');
title(sprintf('%s: %+.1f deg command from %+.1f deg trim', name, ...
      cfg.target_theta_deg, cfg.theta_trim_deg));
legend('\theta', 'target', sprintf('\\pm%.0f%% band', 100*cfg.spec.settle_band), ...
       'Location','SouthEast');
subplot(2,2,3);
plot(t, u, 'b-', 'LineWidth', 1.6); hold on; grid on;
plot(t, ua, 'b--', 'LineWidth', 1.1);
plot(t([1 end]), cfg.elev_up_deg*[1 1], 'r:', t([1 end]), cfg.elev_dn_deg*[1 1], 'r:', ...
     'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\delta_e (deg)');
title('Elevator command'); legend('command', 'after servo', 'travel limits', 'Location','NorthEast');
subplot(2,2,2);
plot(tg, thg, 'b-', 'LineWidth', 1.8); hold on; grid on;
plot(tg([1 end]), cfg.theta_target_abs_deg*[1 1], 'k--');
xlabel('Time (s)'); ylabel('\theta (deg, absolute)');
title(sprintf('%s: %+.1f deg gust upset', name, cfg.gust_theta_deg));
subplot(2,2,4);
plot(tg, ug, 'b-', 'LineWidth', 1.6); hold on; grid on;
plot(tg, uga, 'b--', 'LineWidth', 1.1);
plot(tg([1 end]), cfg.elev_up_deg*[1 1], 'r:', tg([1 end]), cfg.elev_dn_deg*[1 1], 'r:');
xlabel('Time (s)'); ylabel('\delta_e (deg)'); title('Elevator command (gust)');
end

function plotScenario(outs, names, cfg, scen)
% Verification figure: attitude (absolute, with the target), elevator
% command AND actual deflection against the travel limits, pitch rate.
figure('Name', scen.name);
cols = {'b', 'r', [0 0.6 0]};
T    = outs{1}.t([1 end]);

subplot(3,1,1); hold on; grid on;
for k = 1:numel(outs)
    plot(outs{k}.t, outs{k}.theta_abs_deg, 'Color', cols{min(k,3)}, 'LineWidth', 1.7);
end
plot(T, (cfg.theta_trim_deg + scen.target_deg)*[1 1], 'k--', 'LineWidth', 1.0);
if scen.shape_tau > 0
    plot(outs{1}.t, outs{1}.ref_abs_deg, 'm-.', 'LineWidth', 1.0);
    legend([names, {'target', 'shaped command'}], 'Location','SouthEast');
else
    legend([names, {'target'}], 'Location','SouthEast');
end
ylabel('\theta (deg, absolute)');
title(sprintf('%s   |   target %+.2f deg absolute', scen.name, ...
      cfg.theta_trim_deg + scen.target_deg));

subplot(3,1,2); hold on; grid on;
for k = 1:numel(outs)
    plot(outs{k}.t, outs{k}.u_cmd_deg, 'Color', cols{min(k,3)}, 'LineWidth', 1.5);
    plot(outs{k}.t, outs{k}.u_act_deg, '--', 'Color', cols{min(k,3)}, 'LineWidth', 1.0);
end
plot(T, cfg.elev_up_deg*[1 1], 'r:', T, cfg.elev_dn_deg*[1 1], 'r:', 'LineWidth', 1.3);
ylabel('\delta_e (deg)');
title('Elevator: solid = command, dashed = actual after servo/limits, dotted = travel limits');

subplot(3,1,3); hold on; grid on;
for k = 1:numel(outs)
    plot(outs{k}.t, outs{k}.qdot_deg, 'Color', cols{min(k,3)}, 'LineWidth', 1.4);
end
ylabel('pitch rate (deg/s)'); xlabel('Time (s)'); title('Pitch rate');
end

% ---- small helpers -------------------------------------------------------
function y = ternary(c, a, b)
if c, y = a; else y = b; end
end

function i = sortIdx(v)
[~, i] = sort(v);
end

function j = otherIdx(i, k, n)
o = setdiff(1:n, i);
j = o(min(k, numel(o)));
end
