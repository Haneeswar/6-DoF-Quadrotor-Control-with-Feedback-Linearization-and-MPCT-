function res = runSimulation(cfg)
%RUNSIMULATION  Closed loop: position loop every innerSteps samples, attitude MPC every sample.

    Ts = cfg.Ts;
    nIn = cfg.innerSteps;
    nOut = round(cfg.duration/(Ts*nIn)) + 1;
    t = (0:nOut - 1).'*(Ts*nIn);
    ref = buildReference(cfg, t);

    mix = rotorMixer(cfg);

    % initial condition
    x = zeros(12, 1);
    x(7:9) = cfg.startPos(:);
    x(12) = ref.psi(1);
    omega = cfg.omegaStart*ones(4, 1);
    U = inputsFromSpeeds(mix, omega);
    spin = spinSum(omega);

    nFine = 1 + nIn*(nOut - 1);
    states    = zeros(nFine, 12);
    inputs    = zeros(nFine, 4);
    speeds    = zeros(nFine, 4);
    angleRefs = zeros(nFine, 3);
    states(1, :)    = x.';
    inputs(1, :)    = U.';
    speeds(1, :)    = omega.';
    angleRefs(1, :) = x(10:12).';

    row = 1;
    for k = 1:nOut - 1
        [phiRef, thetaRef, thrust] = positionCommand(cfg, ref.pos(k + 1, :), ref.vel(k + 1, :), ...
                                                     ref.acc(k + 1, :), ref.psi(k + 1), x);
        U(1) = thrust;

        % the yaw reference ramps linearly across this position step
        psiRamp = ref.psi(k) + (ref.psi(k + 1) - ref.psi(k))*(0:nIn).'/nIn;

        for i = 1:nIn
            % the window only reaches the end of this position step, so it shrinks
            n = min(cfg.horizon, nIn - i + 1);
            window = [phiRef*ones(n, 1), thetaRef*ones(n, 1), psiRamp(i + 1:i + n)];

            U(2:4) = U(2:4) + attitudeIncrement(cfg, x, spin, U(2:4), window);

            omega = speedsFromInputs(mix, U);
            spin = spinSum(omega);
            x = rk4Step(cfg, x, U, spin);

            row = row + 1;
            states(row, :)    = x.';
            inputs(row, :)    = U.';
            speeds(row, :)    = omega.';
            angleRefs(row, :) = [phiRef, thetaRef, psiRamp(i + 1)];
        end
    end

    res.cfg = cfg;
    res.ref = ref;
    res.tFine = Ts*(0:nFine - 1).';
    res.states = states;
    res.U = inputs;
    res.omega = speeds;
    res.angleRefs = angleRefs;
end
