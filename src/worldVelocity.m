function v = worldVelocity(x)
%WORLDVELOCITY  Inertial velocity (3x1) from the 12-element state.
    v = rotationMatrix(x(10), x(11), x(12))*x(1:3);
end
