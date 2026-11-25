function s = summarizeRun(res, settleTime)
%SUMMARIZERUN  Tracking and actuation figures of a finished run.
%   Errors are evaluated at the position-loop sample times.  The "settled"
%   figures skip the first settleTime seconds (default 10 s), which contain the
%   initial offset between the start point and the reference.

    if nargin < 2
        settleTime = 10;
    end
    cfg = res.cfg;
    pos = res.states(1:cfg.innerSteps:end, 7:9);
    err = sqrt(sum((pos - res.ref.pos).^2, 2));
    late = res.ref.t >= settleTime;

    s.rmse           = sqrt(mean(err.^2));
    s.maxError       = max(err);
    s.rmseSettled    = sqrt(mean(err(late).^2));
    s.maxErrorSettled = max(err(late));
    s.peakThrust     = max(res.U(:, 1));
    w = res.omega(2:end, :);                 % skip the start-up row
    s.rotorRange     = [min(w(:)), max(w(:))];
    s.peakTiltDeg    = max(abs(res.states(:, 10:11)))*180/pi;
end
