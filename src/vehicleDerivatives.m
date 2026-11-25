function dx = vehicleDerivatives(cfg, x, U, spin)
%VEHICLEDERIVATIVES  Time derivative of the 12-state vector.
%   x = [u v w p q r X Y Z phi theta psi]'  (body velocity, body rates,
%   inertial position, Euler angles).  U = [U1 U2 U3 U4]'.

    u = x(1); v = x(2); w = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11);
    Ix = cfg.inertia(1); Iy = cfg.inertia(2); Iz = cfg.inertia(3);
    Jr = cfg.rotorInertia;

    dragAcc = zeros(3, 1);
    if cfg.drag.enabled
        vb = x(1:3);
        dragAcc = 0.5*cfg.drag.rho*(cfg.drag.cd(:).*cfg.drag.area(:)).*vb.*abs(vb)/cfg.mass;
    end

    dx = zeros(12, 1);
    dx(1) = v*r - w*q + cfg.g*sin(theta)              - dragAcc(1);
    dx(2) = w*p - u*r - cfg.g*cos(theta)*sin(phi)     - dragAcc(2);
    dx(3) = u*q - v*p - cfg.g*cos(theta)*cos(phi) + U(1)/cfg.mass - dragAcc(3);
    dx(4) = q*r*(Iy - Iz)/Ix - Jr/Ix*q*spin + U(2)/Ix;
    dx(5) = p*r*(Iz - Ix)/Iy + Jr/Iy*p*spin + U(3)/Iy;
    dx(6) = p*q*(Ix - Iy)/Iz + U(4)/Iz;
    dx(7:9)   = rotationMatrix(phi, theta, x(12))*[u; v; w];
    dx(10:12) = eulerRates(phi, theta, [p; q; r]);
end
