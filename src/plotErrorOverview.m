function plotErrorOverview(list, results, outDir)
%PLOTERROROVERVIEW  Position error against time for every trajectory in one figure.

    fig = figure('Visible', 'off', 'Position', [100 100 1500 1100]);
    nRows = ceil(numel(list)/3);
    for n = 1:numel(list)
        res = results{n};
        pos = res.states(1:res.cfg.innerSteps:end, 7:9);
        err = sqrt(sum((pos - res.ref.pos).^2, 2));
        subplot(nRows, 3, n);
        plot(res.ref.t, err, 'b');
        title(sprintf('Trajectory %d', list(n)));
        xlabel('t [s]'); ylabel('error [m]');
    end
    styleFigure(fig);
    savePng(fig, fullfile(outDir, 'error_all.png'));
    close(fig);
end
