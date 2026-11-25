function du = attitudeIncrement(cfg, x, spin, uPrev, refWindow)
%ATTITUDEINCREMENT  First torque increment of the unconstrained attitude MPC.
%   The attitude model is linear parameter-varying: its matrix is rebuilt at
%   every call from the body rates and the rotor spin sum.
%     model state  z = [phi phi_dot theta theta_dot psi psi_dot]'
%     inputs       U2, U3, U4 (torques), outputs are the three angles
%   refWindow holds the angle references for the next n samples (n x 3).

    n = size(refWindow, 1);
    Ix = cfg.inertia(1); Iy = cfg.inertia(2); Iz = cfg.inertia(3);
    Jr = cfg.rotorInertia;

    rates = eulerRates(x(10), x(11), x(4:6));
    phiD = rates(1); thetaD = rates(2); psiD = rates(3);

    A = zeros(6);
    A(1, 2) = 1;
    A(3, 4) = 1;
    A(5, 6) = 1;
    A(2, 4) = -spin*Jr/Ix;
    A(2, 6) = thetaD*(Iy - Iz)/Ix;
    A(4, 2) =  spin*Jr/Iy;
    A(4, 6) = phiD*(Iz - Ix)/Iy;
    A(6, 2) = 0.5*thetaD*(Ix - Iy)/Iz;   % the yaw coupling product is split
    A(6, 4) = 0.5*phiD*(Ix - Iy)/Iz;     % between the two rate states

    B = zeros(6, 3);
    B(2, 1) = 1/Ix;
    B(4, 2) = 1/Iy;
    B(6, 3) = 1/Iz;

    C = zeros(3, 6);
    C(1, 1) = 1;  C(2, 3) = 1;  C(3, 5) = 1;

    % forward-Euler discretization
    Ad = eye(6) + cfg.Ts*A;
    Bd = cfg.Ts*B;

    % augmented model carries the previous torques: xa = [z; U_prev]
    Aa = [Ad, Bd; zeros(3, 6), eye(3)];
    Ba = [Bd; eye(3)];
    Ca = [C, zeros(3, 3)];
    xa = [x(10); phiD; x(11); thetaD; x(12); psiD; uPrev(:)];

    % output prediction  y = Phi*xa + Gamma*duSequence
    powers = cell(n + 1, 1);
    powers{1} = eye(9);
    for i = 1:n
        powers{i + 1} = powers{i}*Aa;
    end
    Phi   = zeros(3*n, 9);
    Gamma = zeros(3*n, 3*n);
    for i = 1:n
        rows = 3*(i - 1) + (1:3);
        Phi(rows, :) = Ca*powers{i + 1};
        for j = 1:i
            cols = 3*(j - 1) + (1:3);
            Gamma(rows, cols) = Ca*powers{i - j + 1}*Ba;
        end
    end

    Qbar = kron(eye(n), cfg.wOut*eye(3));
    Qbar(end-2:end, end-2:end) = cfg.wFinal*eye(3);
    Rbar = kron(eye(n), cfg.wDu*eye(3));

    refVec = reshape(refWindow.', [], 1);
    H = Gamma.'*Qbar*Gamma + Rbar;
    f = Gamma.'*Qbar*(Phi*xa - refVec);
    duAll = -(H \ f);
    du = duAll(1:3);
end
