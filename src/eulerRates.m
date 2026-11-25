function angleRates = eulerRates(phi, theta, pqr)
%EULERRATES  Body rates [p; q; r] to [phi_dot; theta_dot; psi_dot].
    cf = cos(phi);  sf = sin(phi);
    tt = tan(theta);
    sc = 1/cos(theta);
    T = [1, sf*tt, cf*tt;
         0, cf,    -sf;
         0, sf*sc, cf*sc];
    angleRates = T*pqr(:);
end
