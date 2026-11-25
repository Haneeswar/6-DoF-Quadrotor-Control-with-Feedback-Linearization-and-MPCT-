function s = spinSum(omega)
%SPINSUM  Net rotor spin that drives the gyroscopic term.
    s = omega(1) - omega(2) + omega(3) - omega(4);
end
