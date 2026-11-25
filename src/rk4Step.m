function xNext = rk4Step(cfg, x, U, spin)
%RK4STEP  One fourth-order Runge-Kutta step of length Ts with inputs held constant.
    h  = cfg.Ts;
    k1 = vehicleDerivatives(cfg, x,            U, spin);
    k2 = vehicleDerivatives(cfg, x + 0.5*h*k1, U, spin);
    k3 = vehicleDerivatives(cfg, x + 0.5*h*k2, U, spin);
    k4 = vehicleDerivatives(cfg, x + h*k3,     U, spin);
    xNext = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
