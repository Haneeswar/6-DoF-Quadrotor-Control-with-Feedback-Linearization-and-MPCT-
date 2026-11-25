function [phiRef, thetaRef, thrust] = positionCommand(cfg, refPos, refVel, refAcc, psiRef, x)
%POSITIONCOMMAND  Outer loop: position reference to roll, pitch and thrust.
%   Every axis is a double integrator driven by the commanded acceleration
%       a = refAcc + kp*(refPos - pos) + kd*(refVel - vel)
%   with kp and kd chosen so the error dynamics have the configured poles.

    pos = x(7:9);
    vel = worldVelocity(x);

    kp = real(cfg.poles(:, 1).*cfg.poles(:, 2));
    kd = -real(cfg.poles(:, 1) + cfg.poles(:, 2));
    aCmd = refAcc(:) + kp.*(refPos(:) - pos) + kd.*(refVel(:) - vel);

    lift = aCmd(3) + cfg.g;
    a = aCmd(1)/lift;
    b = aCmd(2)/lift;
    c = cos(psiRef);
    s = sin(psiRef);

    thetaRef = atan(a*c + b*s);
    phiRef   = atan(cos(thetaRef)*(a*s - b*c));
    thrust   = lift*cfg.mass/(cos(phiRef)*cos(thetaRef));
end
