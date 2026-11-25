function runScenarios(cfg)
%RUNSCENARIOS  Run one trajectory, or all nine, and produce the outputs.
%   cfg.trajectory is an integer from 1 to 9, or 'all'.

    batch = ischar(cfg.trajectory) || isstring(cfg.trajectory);
    if batch
        if ~strcmpi(char(cfg.trajectory), 'all')
            error('cfg.trajectory must be an integer from 1 to 9, or ''all''.');
        end
        list = 1:9;
    else
        list = cfg.trajectory;
    end

    if cfg.saveFigures && ~exist(cfg.figureDir, 'dir')
        mkdir(cfg.figureDir);
    end

    results = cell(numel(list), 1);
    stats   = cell(numel(list), 1);
    fprintf('%5s %9s %9s %9s %9s %9s %9s\n', 'traj', 'RMS', 'max', 'RMS>=10s', 'max>=10s', 'thrust', 'tilt');
    for n = 1:numel(list)
        c = cfg;
        c.trajectory = list(n);
        res = runSimulation(c);
        s = summarizeRun(res);
        results{n} = res;
        stats{n} = s;
        fprintf('%5d %9.3f %9.3f %9.3f %9.3f %9.2f %9.1f\n', list(n), s.rmse, s.maxError, ...
                s.rmseSettled, s.maxErrorSettled, s.peakThrust, max(s.peakTiltDeg));

        if batch
            if cfg.saveFigures
                plotResults(res, cfg.figureDir, sprintf('traj%d_', list(n)), false);
            end
        else
            if cfg.animate
                animateFlight(res, cfg.layout);
            end
            if cfg.saveFigures || cfg.showPlots
                saveDir = '';
                if cfg.saveFigures, saveDir = cfg.figureDir; end
                plotResults(res, saveDir, sprintf('traj%d_', list(n)), cfg.showPlots);
            end
        end
    end

    if batch && cfg.saveFigures
        writeSummary(list, stats, cfg.figureDir);
        plotErrorOverview(list, results, cfg.figureDir);
        fprintf('Figures and tables written to %s\n', cfg.figureDir);
    end
end
