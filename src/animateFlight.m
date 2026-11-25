function animateFlight(res, layout, frameStep)
%ANIMATEFLIGHT  3D animation of the flight with one of two side-panel layouts.
%   layout 1: roll/pitch views and the four inputs.  layout 2: position and
%   angle histories.  frameStep > 1 skips frames to play faster.

    if nargin < 2, layout = 1; end
    if nargin < 3, frameStep = 1; end
    sub = 5;                                   % animation frames per sample

    ref = res.ref;
    nF = numel(res.tFine);
    tA = linspace(0, res.tFine(end), (nF - 1)*sub + 1).';
    st = interp1(res.tFine, res.states, tA);
    uIdx = [1; 2 + floor((0:numel(tA) - 2).'/sub)];
    U = res.U(uIdx, :);

    limHi = max(max(ref.pos(:, 1:2)));
    limLo = min(min(ref.pos(:, 1:2)));
    half = 0.15*limHi;                         % half arm length, display only
    bodyX = [half; 0; 0];
    bodyY = [0; half; 0];
    grey = [0.9 0.9 0.9];

    fig = figure('Position', [50 50 1280 720], 'Color', [0.8 0.8 0.8]);
    subplot(4, 3, [1 2 4 5 7 8]);
    plot3(ref.pos(:, 1), ref.pos(:, 2), ref.pos(:, 3), 'b', 'LineWidth', 1); hold on;
    hPath = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 1);
    hArmX = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 5);
    hArmY = plot3(NaN, NaN, NaN, 'g', 'LineWidth', 5);
    set(gca, 'Color', grey);
    xlim([limLo limHi]); ylim([limLo limHi]); zlim([0 max(ref.pos(:, 3))]);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); grid on; view(3);
    legend('reference', 'trajectory', 'drone x', 'drone y', 'Location', 'northwest');

    curves = {};      % growing curves: handle and the series they follow
    series = {};
    if layout == 1
        subplot(4, 3, 10);
        hRoll = plot(NaN, NaN, '--g', 'LineWidth', 2);
        set(gca, 'Color', grey); grid on;
        xlim([-0.9 0.9]*half); ylim([-0.1 0.1]*half);
        title('roll view (Y arm)', 'FontSize', 9);
        subplot(4, 3, 11);
        hPitch = plot(NaN, NaN, '--r', 'LineWidth', 2);
        set(gca, 'Color', grey, 'XDir', 'reverse'); grid on;
        xlim([-0.9 0.9]*half); ylim([-0.1 0.1]*half);
        title('pitch view (X arm)', 'FontSize', 9);
        names = {'Thrust U_1 [N]', 'Roll torque U_2 [N m]', 'Pitch torque U_3 [N m]', 'Yaw torque U_4 [N m]'};
        for i = 1:4
            subplot(4, 3, 3*i);
            h = plot(NaN, NaN, 'b', 'LineWidth', 1);
            set(gca, 'Color', grey); grid on;
            pad = 0.05*max(max(U(:, i)) - min(U(:, i)), 1e-6);
            xlim([0 tA(end)]); ylim([min(U(:, i)) - pad, max(U(:, i)) + pad]);
            ylabel(names{i}, 'FontSize', 8);
            curves{end + 1} = h; series{end + 1} = U(:, i);
        end
        xlabel('t [s]');
    else
        cells = {10, 'X', 7, ref.pos(:, 1), ref.t;
                 11, 'Y', 8, ref.pos(:, 2), ref.t;
                 12, 'Z', 9, ref.pos(:, 3), ref.t;
                 3, 'Roll', 10, res.angleRefs(:, 1), res.tFine;
                 6, 'Pitch', 11, res.angleRefs(:, 2), res.tFine;
                 9, 'Yaw', 12, res.angleRefs(:, 3), res.tFine};
        for i = 1:size(cells, 1)
            subplot(4, 3, cells{i, 1});
            plot(cells{i, 5}, cells{i, 4}, 'b', 'LineWidth', 1); hold on;
            h = plot(NaN, NaN, 'r', 'LineWidth', 1);
            set(gca, 'Color', grey); grid on;
            col = st(:, cells{i, 3});
            pad = 0.05*max(max(col) - min(col), 1e-6);
            xlim([0 tA(end)]); ylim([min(col) - pad, max(col) + pad]);
            ylabel(cells{i, 2});
            if i <= 3, xlabel('t [s]'); end
            curves{end + 1} = h; series{end + 1} = col;
        end
    end

    for j = 1:frameStep:numel(tA)
        c = st(j, 7:9).';
        R = rotationMatrix(st(j, 10), st(j, 11), st(j, 12));
        xEnds = [c - R*bodyX, c + R*bodyX];
        yEnds = [c - R*bodyY, c + R*bodyY];
        set(hArmX, 'XData', xEnds(1, :), 'YData', xEnds(2, :), 'ZData', xEnds(3, :));
        set(hArmY, 'XData', yEnds(1, :), 'YData', yEnds(2, :), 'ZData', yEnds(3, :));
        set(hPath, 'XData', st(1:j, 7), 'YData', st(1:j, 8), 'ZData', st(1:j, 9));
        if layout == 1
            set(hRoll,  'XData', [-0.81 0.81]*half, 'YData', [yEnds(3, 1) - c(3), yEnds(3, 2) - c(3)]);
            set(hPitch, 'XData', [0.81 -0.81]*half, 'YData', [xEnds(3, 2) - c(3), xEnds(3, 1) - c(3)]);
        end
        for i = 1:numel(curves)
            set(curves{i}, 'XData', tA(1:j), 'YData', series{i}(1:j));
        end
        drawnow;
    end
end
