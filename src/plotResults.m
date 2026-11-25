function plotResults(res, saveDir, prefix, show)
%PLOTRESULTS  Trajectory, position/velocity, attitude and actuation figures.
%   saveDir  folder for PNG files (empty or omitted: do not save)
%   prefix   file-name prefix, e.g. 'traj1_' (default '')
%   show     keep the figures open on screen (default: true when not saving)

    if nargin < 2, saveDir = ''; end
    if nargin < 3, prefix = ''; end
    doSave = ~isempty(saveDir);
    if nargin < 4, show = ~doSave; end
    vis = 'off';
    if show, vis = 'on'; end
    if doSave && ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end

    ref = res.ref;
    t = res.tFine;
    st = res.states;
    vel = zeros(size(st, 1), 3);
    for i = 1:size(st, 1)
        vel(i, :) = worldVelocity(st(i, :).').';
    end
    axisName = {'X', 'Y', 'Z'};

    % 3D path
    f1 = figure('Visible', vis, 'Position', [100 100 900 720]);
    plot3(ref.pos(:, 1), ref.pos(:, 2), ref.pos(:, 3), 'b', 'LineWidth', 1.2); hold on;
    plot3(st(:, 7), st(:, 8), st(:, 9), 'r', 'LineWidth', 1);
    
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    legend('reference', 'trajectory', 'Location', 'northeast');
    finish(f1, doSave, show, saveDir, [prefix 'path3d']);

    % position and velocity
    f2 = figure('Visible', vis, 'Position', [100 100 1300 950]);
    for a = 1:3
        subplot(3, 2, 2*a - 1);
        plot(ref.t, ref.pos(:, a), 'b', t, st(:, 6 + a), 'r', 'LineWidth', 1); 
        ylabel([axisName{a} ' [m]']);
        if a == 1, legend('reference', 'actual', 'Location', 'best'); end
        if a == 3, xlabel('t [s]'); end
        subplot(3, 2, 2*a);
        plot(ref.t, ref.vel(:, a), 'b', t, vel(:, a), 'r', 'LineWidth', 1); 
        ylabel([axisName{a} '-velocity [m/s]']);
        if a == 3, xlabel('t [s]'); end
    end
    finish(f2, doSave, show, saveDir, [prefix 'position_velocity']);

    % attitude
    f3 = figure('Visible', vis, 'Position', [100 100 1100 900]);
    angleName = {'Roll \phi [rad]', 'Pitch \theta [rad]', 'Yaw \psi [rad]'};
    for a = 1:3
        subplot(3, 1, a);
        plot(t, res.angleRefs(:, a), 'b', t, st(:, 9 + a), 'r', 'LineWidth', 1); 
        ylabel(angleName{a});
        if a == 1, legend('reference', 'actual', 'Location', 'best'); end
    end
    xlabel('t [s]');
    finish(f3, doSave, show, saveDir, [prefix 'attitude']);

    % control inputs and rotor speeds
    f4 = figure('Visible', vis, 'Position', [100 100 1300 950]);
    inName = {'U_1 [N]', 'U_2 [N m]', 'U_3 [N m]', 'U_4 [N m]'};
    for a = 1:4
        subplot(4, 2, 2*a - 1);
        plot(t, res.U(:, a), 'b', 'LineWidth', 1); ylabel(inName{a});
        if a == 4, xlabel('t [s]'); end
        subplot(4, 2, 2*a);
        plot(t, res.omega(:, a), 'b', 'LineWidth', 1); 
        ylabel(sprintf('\\omega_%d [rad/s]', a));
        if a == 4, xlabel('t [s]'); end
    end
    finish(f4, doSave, show, saveDir, [prefix 'inputs_rotors']);
end

function finish(fig, doSave, show, saveDir, name)
    styleFigure(fig);
    if doSave
        savePng(fig, fullfile(saveDir, [name '.png']));
    end
    if ~show
        close(fig);
    end
end
