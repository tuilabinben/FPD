%% ========================================================================
%  Compare_Angular_Motion_Profiles.m
%
%  COMPARISON OF ANGULAR MOTION PROFILES
%
%  This program compares three motion-planning methods:
%
%       1. Trapezoidal Velocity Profile
%       2. S-Curve Velocity Profile
%       3. Pure S-Curve Velocity Profile
%
%  All three methods use the SAME physical input parameters:
%
%       theta0      : Initial angular position
%       thetaf      : Final angular position
%       omegaMax    : Maximum angular velocity
%       alphaMax    : Maximum angular acceleration
%
%  Output:
%
%       Figure 1 - Trapezoidal profile
%       Figure 2 - S-Curve profile
%       Figure 3 - Pure S-Curve profile
%
%  Each figure contains:
%
%       Angular Position
%       Angular Velocity
%       Angular Acceleration
%       Angular Jerk
%
% ========================================================================

clear;
clc;
close all;


%% ========================================================================
% 1. COMMON INPUT PARAMETERS
% ========================================================================

% ------------------------------------------------------------------------
% Angular position
% ------------------------------------------------------------------------

theta0 = 0;            % Initial angular position [deg]
thetaf = 180;          % Target angular position  [deg]


% ------------------------------------------------------------------------
% Motion limits
% ------------------------------------------------------------------------

omegaMax = 60;         % Maximum angular velocity     [deg/s]
alphaMax = 120;        % Maximum angular acceleration [deg/s^2]


% ------------------------------------------------------------------------
% Simulation time step
% ------------------------------------------------------------------------

dt = 0.001;            % Time step [s]


%% ========================================================================
% 2. S-CURVE SHAPE PARAMETER
% ========================================================================

% rS determines the smoothing level of the normal S-Curve.
%
%       rS -> 0      : approaches Trapezoidal profile
%
%       0 < rS < 1   : normal 7-phase S-Curve
%
%       rS = 1       : Pure S-Curve
%
% Example:
%
%       rS = 0.5
%
% means that the jerk-transition region occupies 50% of the
% acceleration-design parameter.

rS = 0.5;


%% ========================================================================
% 3. CHECK INPUT PARAMETERS
% ========================================================================

if omegaMax <= 0
    error('omegaMax must be greater than zero.');
end

if alphaMax <= 0
    error('alphaMax must be greater than zero.');
end

if dt <= 0
    error('dt must be greater than zero.');
end

if rS <= 0 || rS >= 1
    error('For normal S-Curve, rS must satisfy 0 < rS < 1.');
end


%% ========================================================================
% 4. CALCULATE ANGULAR DISPLACEMENT
% ========================================================================

DeltaTheta = thetaf - theta0;

Theta = abs(DeltaTheta);


% Determine rotation direction
%
% direction = +1 : positive rotation
% direction = -1 : negative rotation

direction = sign(DeltaTheta);


if Theta == 0
    error('theta0 and thetaf must be different.');
end


%% ========================================================================
% 5. GENERATE TRAPEZOIDAL VELOCITY PROFILE
% ========================================================================

trap = generateTrapezoidalAngular( ...
    Theta, ...
    omegaMax, ...
    alphaMax, ...
    dt);


%% ========================================================================
% 6. GENERATE NORMAL S-CURVE VELOCITY PROFILE
% ========================================================================

scurve = generateSCurveAngular( ...
    Theta, ...
    omegaMax, ...
    alphaMax, ...
    rS, ...
    dt);


%% ========================================================================
% 7. GENERATE PURE S-CURVE VELOCITY PROFILE
% ========================================================================

% Pure S-Curve:
%
%       r = 1
%
% Therefore the constant-acceleration interval disappears.

pureS = generateSCurveAngular( ...
    Theta, ...
    omegaMax, ...
    alphaMax, ...
    1.0, ...
    dt);


%% ========================================================================
% 8. CONVERT RELATIVE MOTION TO ABSOLUTE ANGULAR POSITION
% ========================================================================

% ------------------------------------------------------------------------
% Trapezoidal
% ------------------------------------------------------------------------

trap.theta = theta0 + direction * trap.s;

trap.omega = direction * trap.v;

trap.alpha = direction * trap.a;


% ------------------------------------------------------------------------
% Normal S-Curve
% ------------------------------------------------------------------------

scurve.theta = theta0 + direction * scurve.s;

scurve.omega = direction * scurve.v;

scurve.alpha = direction * scurve.a;

scurve.jerk = direction * scurve.j;


% ------------------------------------------------------------------------
% Pure S-Curve
% ------------------------------------------------------------------------

pureS.theta = theta0 + direction * pureS.s;

pureS.omega = direction * pureS.v;

pureS.alpha = direction * pureS.a;

pureS.jerk = direction * pureS.j;


%% ========================================================================
% 9. DISPLAY INPUT PARAMETERS
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                COMMON MOTION INPUTS\n');
fprintf('===============================================================\n');

fprintf('Initial angle               = %.4f deg\n', theta0);
fprintf('Target angle                = %.4f deg\n', thetaf);
fprintf('Angular displacement        = %.4f deg\n', DeltaTheta);

fprintf('Maximum angular velocity    = %.4f deg/s\n', omegaMax);
fprintf('Maximum angular acceleration= %.4f deg/s^2\n', alphaMax);

fprintf('S-Curve ratio rS            = %.4f\n', rS);


%% ========================================================================
% 10. DISPLAY TRAPEZOIDAL PARAMETERS
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('             TRAPEZOIDAL VELOCITY PROFILE\n');
fprintf('===============================================================\n');

fprintf('Peak angular velocity = %.4f deg/s\n', trap.Vp);

fprintf('Acceleration time     = %.4f s\n', trap.ta);

fprintf('Constant speed time   = %.4f s\n', trap.tv);

fprintf('Deceleration time     = %.4f s\n', trap.ta);

fprintf('Total motion time     = %.4f s\n', trap.T);

fprintf('Peak angular accel.   = %.4f deg/s^2\n', alphaMax);

fprintf('Theoretical jerk      = Infinite at transitions\n');


%% ========================================================================
% 11. DISPLAY NORMAL S-CURVE PARAMETERS
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                 S-CURVE PROFILE\n');
fprintf('===============================================================\n');

fprintf('S-Curve ratio rS      = %.4f\n', rS);

fprintf('Peak angular velocity = %.4f deg/s\n', scurve.Vp);

fprintf('Jerk transition tJ    = %.4f s\n', scurve.tJ);

fprintf('Constant accel. tA    = %.4f s\n', scurve.tA);

fprintf('Constant speed tV     = %.4f s\n', scurve.tV);

fprintf('Peak angular accel.   = %.4f deg/s^2\n', alphaMax);

fprintf('Maximum angular jerk  = %.4f deg/s^3\n', scurve.J);

fprintf('Total motion time     = %.4f s\n', scurve.T);


%% ========================================================================
% 12. DISPLAY PURE S-CURVE PARAMETERS
% ========================================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf('                PURE S-CURVE PROFILE\n');
fprintf('===============================================================\n');

fprintf('Peak angular velocity = %.4f deg/s\n', pureS.Vp);

fprintf('Jerk transition tJ    = %.4f s\n', pureS.tJ);

fprintf('Constant accel. tA    = %.4f s\n', pureS.tA);

fprintf('Constant speed tV     = %.4f s\n', pureS.tV);

fprintf('Peak angular accel.   = %.4f deg/s^2\n', alphaMax);

fprintf('Maximum angular jerk  = %.4f deg/s^3\n', pureS.J);

fprintf('Total motion time     = %.4f s\n', pureS.T);


%% ========================================================================
% 13. CREATE COMPARISON TABLE
% ========================================================================

Method = {
    'Trapezoidal'
    'S-Curve'
    'Pure S-Curve'
    };


TotalTime = [
    trap.T
    scurve.T
    pureS.T
    ];


PeakAngularVelocity = [
    trap.Vp
    scurve.Vp
    pureS.Vp
    ];


PeakAngularAcceleration = [
    max(abs(trap.alpha))
    max(abs(scurve.alpha))
    max(abs(pureS.alpha))
    ];


PeakAngularJerk = [
    Inf
    scurve.J
    pureS.J
    ];


Results = table( ...
    Method, ...
    TotalTime, ...
    PeakAngularVelocity, ...
    PeakAngularAcceleration, ...
    PeakAngularJerk);


fprintf('\n');
fprintf('===============================================================\n');
fprintf('                COMPARISON RESULTS\n');
fprintf('===============================================================\n');

disp(Results);



%% ========================================================================
% 14. FIGURE 1
% TRAPEZOIDAL VELOCITY PROFILE
% ========================================================================

figure(1);

set(gcf, ...
    'Name','Trapezoidal Angular Motion Profile', ...
    'Color','w', ...
    'Position',[100 30 850 900]);


tiledlayout(4,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');


% ------------------------------------------------------------------------
% 14.1 Angular Position
% ------------------------------------------------------------------------

nexttile;

plot( ...
    trap.t, ...
    trap.theta, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\theta [deg]');

title('Angular Position');

xlim([0 trap.T]);


% ------------------------------------------------------------------------
% 14.2 Angular Velocity
% ------------------------------------------------------------------------

nexttile;

plot( ...
    trap.t, ...
    trap.omega, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\omega [deg/s]');

title('Angular Velocity');

xlim([0 trap.T]);


% ------------------------------------------------------------------------
% 14.3 Angular Acceleration
% ------------------------------------------------------------------------

nexttile;

plot( ...
    trap.t, ...
    trap.alpha, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\alpha [deg/s^2]');

title('Angular Acceleration');

xlim([0 trap.T]);


% ------------------------------------------------------------------------
% 14.4 Angular Jerk
%
% In an ideal trapezoidal profile, acceleration changes instantaneously.
%
% Therefore:
%
%           jerk -> infinity
%
% Instead of plotting a numerical derivative, impulses are represented
% symbolically, similarly to the theoretical motion-profile diagram.
% ------------------------------------------------------------------------

nexttile;

hold on;
grid on;
box on;


% Zero jerk baseline
plot( ...
    [0 trap.T], ...
    [0 0], ...
    'LineWidth',1.2);


% Arbitrary height used only to visualize theoretical impulses
Jdisplay = alphaMax * 2;


% Transition times
t1 = 0;
t2 = trap.ta;
t3 = trap.ta + trap.tv;
t4 = trap.T;


% +infinity at start
plot( ...
    [t1 t1], ...
    [0 Jdisplay], ...
    'LineWidth',1.6);

text( ...
    t1, ...
    Jdisplay, ...
    '+\infty', ...
    'VerticalAlignment','bottom');


% -infinity after acceleration
plot( ...
    [t2 t2], ...
    [0 -Jdisplay], ...
    'LineWidth',1.6);

text( ...
    t2, ...
    -Jdisplay, ...
    '-\infty', ...
    'VerticalAlignment','top');


% -infinity when deceleration begins
plot( ...
    [t3 t3], ...
    [0 -Jdisplay], ...
    'LineWidth',1.6);

text( ...
    t3, ...
    -Jdisplay, ...
    '-\infty', ...
    'VerticalAlignment','top');


% +infinity when deceleration ends
plot( ...
    [t4 t4], ...
    [0 Jdisplay], ...
    'LineWidth',1.6);

text( ...
    t4, ...
    Jdisplay, ...
    '+\infty', ...
    'HorizontalAlignment','right', ...
    'VerticalAlignment','bottom');


xlabel('Time [s]');

ylabel('j_\theta');

title('Angular Jerk');

xlim([0 trap.T]);

ylim([ ...
    -1.3*Jdisplay ...
     1.3*Jdisplay]);


sgtitle( ...
    'Trapezoidal Angular Velocity Profile', ...
    'FontWeight','bold');



%% ========================================================================
% 15. FIGURE 2
% NORMAL S-CURVE VELOCITY PROFILE
% ========================================================================

figure(2);

set(gcf, ...
    'Name','S-Curve Angular Motion Profile', ...
    'Color','w', ...
    'Position',[150 30 850 900]);


tiledlayout(4,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');


% ------------------------------------------------------------------------
% 15.1 Angular Position
% ------------------------------------------------------------------------

nexttile;

plot( ...
    scurve.t, ...
    scurve.theta, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\theta [deg]');

title('Angular Position');

xlim([0 scurve.T]);


% ------------------------------------------------------------------------
% 15.2 Angular Velocity
% ------------------------------------------------------------------------

nexttile;

plot( ...
    scurve.t, ...
    scurve.omega, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\omega [deg/s]');

title('Angular Velocity');

xlim([0 scurve.T]);


% ------------------------------------------------------------------------
% 15.3 Angular Acceleration
% ------------------------------------------------------------------------

nexttile;

plot( ...
    scurve.t, ...
    scurve.alpha, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\alpha [deg/s^2]');

title('Angular Acceleration');

xlim([0 scurve.T]);


% ------------------------------------------------------------------------
% 15.4 Angular Jerk
% ------------------------------------------------------------------------

nexttile;

plot( ...
    scurve.t, ...
    scurve.jerk, ...
    'LineWidth',1.8);

grid on;
box on;

xlabel('Time [s]');

ylabel('j_\theta [deg/s^3]');

title('Angular Jerk');

xlim([0 scurve.T]);


sgtitle( ...
    sprintf('S-Curve Angular Velocity Profile   (r_S = %.2f)',rS), ...
    'FontWeight','bold');



%% ========================================================================
% 16. FIGURE 3
% PURE S-CURVE VELOCITY PROFILE
% ========================================================================

figure(3);

set(gcf, ...
    'Name','Pure S-Curve Angular Motion Profile', ...
    'Color','w', ...
    'Position',[200 30 850 900]);


tiledlayout(4,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');


% ------------------------------------------------------------------------
% 16.1 Angular Position
% ------------------------------------------------------------------------

nexttile;

plot( ...
    pureS.t, ...
    pureS.theta, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\theta [deg]');

title('Angular Position');

xlim([0 pureS.T]);


% ------------------------------------------------------------------------
% 16.2 Angular Velocity
% ------------------------------------------------------------------------

nexttile;

plot( ...
    pureS.t, ...
    pureS.omega, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\omega [deg/s]');

title('Angular Velocity');

xlim([0 pureS.T]);


% ------------------------------------------------------------------------
% 16.3 Angular Acceleration
% ------------------------------------------------------------------------

nexttile;

plot( ...
    pureS.t, ...
    pureS.alpha, ...
    'LineWidth',1.8);

grid on;
box on;

ylabel('\alpha [deg/s^2]');

title('Angular Acceleration');

xlim([0 pureS.T]);


% ------------------------------------------------------------------------
% 16.4 Angular Jerk
% ------------------------------------------------------------------------

nexttile;

plot( ...
    pureS.t, ...
    pureS.jerk, ...
    'LineWidth',1.8);

grid on;
box on;

xlabel('Time [s]');

ylabel('j_\theta [deg/s^3]');

title('Angular Jerk');

xlim([0 pureS.T]);


sgtitle( ...
    'Pure S-Curve Angular Velocity Profile', ...
    'FontWeight','bold');



%% ========================================================================
% 17. LOCAL FUNCTION
% TRAPEZOIDAL ANGULAR VELOCITY PROFILE
% ========================================================================

function P = generateTrapezoidalAngular(Theta, omegaMax, alphaMax, dt)

    % ====================================================================
    % Determine whether maximum velocity can be reached.
    %
    % Distance required for full trapezoidal motion:
    %
    %                Theta_min = omegaMax^2 / alphaMax
    %
    % If:
    %
    %       Theta >= Theta_min
    %
    % the motion contains:
    %
    %       acceleration
    %       constant velocity
    %       deceleration
    %
    % Otherwise, the profile becomes triangular.
    % ====================================================================


    ThetaMin = omegaMax^2 / alphaMax;


    if Theta >= ThetaMin

        % ---------------------------------------------------------------
        % Full trapezoidal profile
        % ---------------------------------------------------------------

        Vp = omegaMax;

        ta = Vp / alphaMax;

        tv = Theta / Vp - ta;


    else

        % ---------------------------------------------------------------
        % Triangular profile
        % ---------------------------------------------------------------

        Vp = sqrt(Theta * alphaMax);

        ta = Vp / alphaMax;

        tv = 0;

    end


    % Total motion time
    T = 2*ta + tv;


    % Time vector
    N = ceil(T/dt) + 1;

    t = linspace(0,T,N);


    % Initialize arrays
    s = zeros(size(t));

    v = zeros(size(t));

    a = zeros(size(t));


    % Angular position at the end of acceleration
    thetaA = 0.5 * alphaMax * ta^2;


    % ====================================================================
    % Calculate the motion profile
    % ====================================================================

    for k = 1:length(t)

        tk = t(k);


        % ---------------------------------------------------------------
        % Phase 1
        % Acceleration
        % ---------------------------------------------------------------

        if tk <= ta

            a(k) = alphaMax;

            v(k) = alphaMax * tk;

            s(k) = ...
                0.5 * ...
                alphaMax * ...
                tk^2;


        % ---------------------------------------------------------------
        % Phase 2
        % Constant angular velocity
        % ---------------------------------------------------------------

        elseif tk <= ta + tv

            tau = tk - ta;

            a(k) = 0;

            v(k) = Vp;

            s(k) = ...
                thetaA + ...
                Vp * tau;


        % ---------------------------------------------------------------
        % Phase 3
        % Deceleration
        % ---------------------------------------------------------------

        else

            tau = ...
                tk - ...
                (ta + tv);

            a(k) = -alphaMax;

            v(k) = ...
                Vp - ...
                alphaMax*tau;

            s(k) = ...
                thetaA + ...
                Vp*tv + ...
                Vp*tau - ...
                0.5*alphaMax*tau^2;

        end

    end


    % Remove numerical final-state errors
    s(end) = Theta;

    v(end) = 0;

    a(end) = 0;


    % Store output
    P.t = t;

    P.s = s;

    P.v = v;

    P.a = a;

    P.Vp = Vp;

    P.ta = ta;

    P.tv = tv;

    P.T = T;

end



%% ========================================================================
% 18. LOCAL FUNCTION
% 7-PHASE S-CURVE ANGULAR VELOCITY PROFILE
%
% r controls the profile shape:
%
%       0 < r < 1     Normal S-Curve
%
%       r = 1         Pure S-Curve
%
% ========================================================================

function P = generateSCurveAngular(Theta, omegaMax, alphaMax, r, dt)

    % ====================================================================
    % TIME PARAMETERS
    %
    % Jerk transition:
    %
    %               tJ = r * omegaPeak / alphaMax
    %
    % Constant acceleration:
    %
    %               tA = (1-r)*omegaPeak / alphaMax
    %
    % Total acceleration region:
    %
    %               Ta = 2*tJ + tA
    %
    % ====================================================================


    % --------------------------------------------------------------------
    % Determine whether omegaMax can be reached
    % --------------------------------------------------------------------

    TaMax = ...
        (1+r) * ...
        omegaMax / ...
        alphaMax;


    ThetaMin = ...
        omegaMax * ...
        TaMax;


    if Theta >= ThetaMin

        % Maximum angular velocity can be reached

        Vp = omegaMax;

    else

        % Short angular motion:
        % calculate achievable peak angular velocity.

        Vp = sqrt( ...
            Theta * ...
            alphaMax / ...
            (1+r));

    end


    % --------------------------------------------------------------------
    % Calculate S-Curve timing parameters
    % --------------------------------------------------------------------

    tJ = ...
        r * ...
        Vp / ...
        alphaMax;


    tA = ...
        (1-r) * ...
        Vp / ...
        alphaMax;


    Ta = ...
        2*tJ + ...
        tA;


    % --------------------------------------------------------------------
    % Constant velocity duration
    % --------------------------------------------------------------------

    tV = ...
        Theta/Vp - ...
        Ta;


    if tV < 1e-12
        tV = 0;
    end


    % --------------------------------------------------------------------
    % Angular jerk magnitude
    % --------------------------------------------------------------------

    J = ...
        alphaMax / ...
        tJ;


    % ====================================================================
    % SEVEN S-CURVE PHASES
    %
    % Phase 1 : +J
    % Phase 2 :  0       -> constant +alpha
    % Phase 3 : -J
    % Phase 4 :  0       -> constant omega
    % Phase 5 : -J
    % Phase 6 :  0       -> constant -alpha
    % Phase 7 : +J
    % ====================================================================


    durations = [
        tJ
        tA
        tJ
        tV
        tJ
        tA
        tJ
        ];


    jerks = [
         J
         0
        -J
         0
        -J
         0
         J
        ];


    % Total motion time
    T = sum(durations);


    % Time vector
    N = ceil(T/dt) + 1;

    t = linspace(0,T,N);


    % ====================================================================
    % CALCULATE INITIAL STATE OF EACH PHASE
    % ====================================================================

    nSeg = length(durations);


    s0 = zeros(nSeg,1);

    v0 = zeros(nSeg,1);

    a0 = zeros(nSeg,1);


    for k = 2:nSeg

        h = durations(k-1);

        j = jerks(k-1);


        % Acceleration
        a0(k) = ...
            a0(k-1) + ...
            j*h;


        % Angular velocity
        v0(k) = ...
            v0(k-1) + ...
            a0(k-1)*h + ...
            0.5*j*h^2;


        % Angular position
        s0(k) = ...
            s0(k-1) + ...
            v0(k-1)*h + ...
            0.5*a0(k-1)*h^2 + ...
            (1/6)*j*h^3;

    end


    % ====================================================================
    % PHASE START AND END TIMES
    % ====================================================================

    tStart = [
        0
        cumsum(durations(1:end-1))
        ];


    tEnd = ...
        cumsum(durations);


    % ====================================================================
    % INITIALIZE OUTPUT ARRAYS
    % ====================================================================

    s = zeros(size(t));

    v = zeros(size(t));

    a = zeros(size(t));

    jerk = zeros(size(t));


    % ====================================================================
    % CALCULATE MOTION AT EACH TIME SAMPLE
    % ====================================================================

    for n = 1:length(t)

        tn = t(n);


        % Find active S-Curve phase
        k = find( ...
            (tn <= tEnd + 1e-12) & ...
            (durations > 1e-14), ...
            1, ...
            'first');


        if isempty(k)

            k = nSeg;

        end


        % Local time inside current phase
        tau = ...
            tn - ...
            tStart(k);


        % Prevent numerical round-off errors
        tau = ...
            max( ...
                0, ...
                min(tau,durations(k)));


        j = jerks(k);


        % ---------------------------------------------------------------
        % Angular jerk
        % ---------------------------------------------------------------

        jerk(n) = j;


        % ---------------------------------------------------------------
        % Angular acceleration
        %
        % alpha(t) = alpha0 + j*t
        % ---------------------------------------------------------------

        a(n) = ...
            a0(k) + ...
            j*tau;


        % ---------------------------------------------------------------
        % Angular velocity
        %
        % omega(t) =
        %
        % omega0
        % + alpha0*t
        % + 1/2*j*t^2
        % ---------------------------------------------------------------

        v(n) = ...
            v0(k) + ...
            a0(k)*tau + ...
            0.5*j*tau^2;


        % ---------------------------------------------------------------
        % Angular position
        %
        % theta(t) =
        %
        % theta0
        % + omega0*t
        % + 1/2*alpha0*t^2
        % + 1/6*j*t^3
        % ---------------------------------------------------------------

        s(n) = ...
            s0(k) + ...
            v0(k)*tau + ...
            0.5*a0(k)*tau^2 + ...
            (1/6)*j*tau^3;

    end


    % ====================================================================
    % REMOVE SMALL NUMERICAL ERRORS
    % ====================================================================

    s(end) = Theta;

    v(end) = 0;

    a(end) = 0;

    jerk(end) = 0;


    % ====================================================================
    % STORE RESULTS
    % ====================================================================

    P.t = t;

    P.s = s;

    P.v = v;

    P.a = a;

    P.j = jerk;


    P.Vp = Vp;


    P.tJ = tJ;

    P.tA = tA;

    P.tV = tV;


    P.J = J;

    P.T = T;

end