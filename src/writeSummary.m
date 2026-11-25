function writeSummary(list, stats, outDir)
%WRITESUMMARY  Tracking and actuation metrics as a CSV file and a LaTeX table body.

    fid = fopen(fullfile(outDir, 'trajectory_summary.csv'), 'w');
    fprintf(fid, 'trajectory,rmse_m,max_error_m,rmse_settled_m,max_error_settled_m,peak_thrust_N,omega_min,omega_max,peak_roll_deg,peak_pitch_deg\n');
    ft = fopen(fullfile(outDir, 'summary_table.tex'), 'w');
    for n = 1:numel(list)
        s = stats{n};
        fprintf(fid, '%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.1f,%.1f,%.2f,%.2f\n', list(n), s.rmse, s.maxError, ...
                s.rmseSettled, s.maxErrorSettled, s.peakThrust, s.rotorRange(1), s.rotorRange(2), ...
                s.peakTiltDeg(1), s.peakTiltDeg(2));
        if n < numel(list), ending = ' \\\\\n'; else, ending = '\n'; end   % last row: no line break
        fprintf(ft, ['%d & %.3f & %.3f & %.3f & %.3f & %.2f & %.0f--%.0f & %.2f & %.2f' ending], list(n), ...
                s.rmse, s.maxError, s.rmseSettled, s.maxErrorSettled, s.peakThrust, ...
                s.rotorRange(1), s.rotorRange(2), s.peakTiltDeg(1), s.peakTiltDeg(2));
    end
    fclose(fid);
    fclose(ft);
end
