function cfg = defaultSettings()
%DEFAULTSETTINGS  Vehicle data, controller tuning and scenario options.

    rpmToRad2 = (60/(2*pi))^2;

    % airframe
    cfg.mass         = 0.698;                 % kg
    cfg.g            = 9.81;                  % m/s^2
    cfg.inertia      = [0.0034 0.0034 0.006]; % [Ix Iy Iz], kg m^2
    cfg.rotorInertia = 1.302e-6;              % gyroscopic coupling term
    cfg.armLength    = 0.171;                 % m, centre to rotor
    cfg.thrustCoeff  = 7.6184e-8*rpmToRad2;   % N s^2
    cfg.torqueCoeff  = 2.6839e-9*rpmToRad2;   % N m s^2
    cfg.omegaStart   = 110*pi/3;              % rad/s, every rotor at t = -Ts

    % optional aerodynamic drag, body axes
    side = 2*cfg.armLength*0.01;
    body = 0.05^2;
    cfg.drag.enabled = false;
    cfg.drag.cd      = [1.5 1.5 2.0];
    cfg.drag.area    = [side+body, side+body, 2*side+body];   % cross-sections along u, v, w
    cfg.drag.rho     = 1.225;                                 % kg/m^3

    % timing
    cfg.Ts         = 0.1;    % attitude loop step, s
    cfg.innerSteps = 4;      % attitude steps per position step
    cfg.duration   = 100;    % s

    % attitude MPC
    cfg.horizon = 4;
    cfg.wOut    = 10;        % weight on angle error
    cfg.wFinal  = 20;        % weight on the last predicted angle error
    cfg.wDu     = 10;        % weight on torque increments

    % position loop: two poles per axis (rows x, y, z)
    cfg.poles = [-0.5+0.3i, -0.5-0.3i;
                 -0.5+0.3i, -0.5-0.3i;
                 -1.0+1.3i, -1.0-1.3i];

    % reference
    cfg.trajectory = 1;      % 1 ... 9, or 'all' to run every trajectory
    cfg.ref.radius = 2;      % m
    cfg.ref.freq   = 0.025;  % Hz
    cfg.ref.zStart = 5;      % m
    cfg.ref.zEnd   = 25;     % m
    cfg.startPos   = [0 -1 0];

    % output
    cfg.layout       = 1;        % animation panels, 1 or 2
    cfg.animate      = true;     % 3D animation (single trajectory only)
    cfg.showPlots    = true;     % open the figures on screen (single trajectory only)
    cfg.saveFigures  = true;     % write the figures and tables to figureDir
    cfg.figureDir    = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'report', 'figures');
end
