function ref = buildReference(cfg, t)
%BUILDREFERENCE  Position, velocity, acceleration and yaw along the chosen trajectory.
%   t is a column of sample times; every output has one row per sample.

    r = cfg.ref.radius;
    w = 2*pi*cfg.ref.freq;
    n = numel(t);
    rise = (cfg.ref.zEnd - cfg.ref.zStart)/t(end);
    k = cfg.trajectory;

    % horizontal motion
    switch k
        case {1, 2, 3, 4, 9}
            [xy, vxy, axy] = circleMotion(t, r, w);
        case 5
            [xy, vxy, axy] = spiralMotion(t, r, w, r/15, 1);
        case 6
            [xy, vxy, axy] = spiralMotion(t, r, w, r/500, 2);
        case 7
            xy  = [t/10 + 1, t/10 - 2];
            vxy = 0.1*ones(n, 2);
            axy = zeros(n, 2);
        case 8
            xy  = [r/5*sin(w*t) + t/100, t/100 - 1];
            vxy = [r/5*w*cos(w*t) + 0.01, 0.01*ones(n, 1)];
            axy = [-r/5*w^2*sin(w*t), zeros(n, 1)];
        otherwise
            error('Trajectory must be an integer from 1 to 9.');
    end

    % vertical motion
    z  = cfg.ref.zStart + rise*t;
    vz = rise*ones(n, 1);
    az = zeros(n, 1);
    if k == 9
        amp = 7*rise;
        z  = cfg.ref.zStart + amp*sin(t);
        vz = amp*cos(t);
        az = -amp*sin(t);
    end

    % trajectories 2-4: circle for the first 100 steps, then a straight run
    if any(k == [2 3 4])
        k0   = 101;
        tail = (k0 + 1):n;
        tau  = t(tail) - t(k0);
        for c = 1:2
            xy(tail, c)  = xy(k0, c) + 0.1*tau;
            vxy(tail, c) = 0.1;
            axy(tail, c) = 0;
        end
        switch k
            case 2
                z(tail)  = z(k0) + rise*tau;
                vz(tail) = rise;
                az(tail) = 0;
            case 3
                z(tail)  = z(k0) + rise*tau.^2;
                vz(tail) = 2*rise*tau;
                az(tail) = 2*rise;
            case 4
                z(tail)  = z(k0) + 50*rise*sin(0.1*tau);
                vz(tail) = 5*rise*cos(0.1*tau);
                az(tail) = -0.5*rise*sin(0.1*tau);
        end
    end

    ref.t   = t;
    ref.pos = [xy, z];
    ref.vel = [vxy, vz];
    ref.acc = [axy, az];
    ref.psi = headingAngle(ref.pos);
end

function [xy, vxy, axy] = circleMotion(t, r, w)
    c = cos(w*t);
    s = sin(w*t);
    xy  = [r*c, r*s];
    vxy = [-r*w*s, r*w*c];
    axy = [-r*w^2*c, -r*w^2*s];
end

function [xy, vxy, axy] = spiralMotion(t, r, w, rate, p)
%   radius grows as rate*t^p + r
    c = cos(w*t);
    s = sin(w*t);
    rad = rate*t.^p + r;
    d1  = p*rate*t.^(p - 1);
    if p > 1
        d2 = p*(p - 1)*rate*t.^(p - 2);
    else
        d2 = zeros(size(t));
    end
    xy  = [rad.*c, rad.*s];
    vxy = [d1.*c - rad*w.*s, d1.*s + rad*w.*c];
    axy = [d2.*c - 2*d1*w.*s - rad*w^2.*c, ...
           d2.*s + 2*d1*w.*c - rad*w^2.*s];
end

function psi = headingAngle(pos)
%   direction of travel between samples, unwrapped
    step = diff(pos(:, 1:2), 1, 1);
    step = [step(1, :); step];
    psi = atan2(step(:, 2), step(:, 1));
    psi(1) = atan2(pos(1, 2), pos(1, 1)) + pi/2;
    psi = unwrap(psi);
end
