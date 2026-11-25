function R = rotationMatrix(phi, theta, psi)
%ROTATIONMATRIX  Body-to-inertial rotation for roll phi, pitch theta, yaw psi (Z-Y-X).
    cf = cos(phi);   sf = sin(phi);
    ct = cos(theta); st = sin(theta);
    cy = cos(psi);   sy = sin(psi);
    R = [ct*cy, sf*st*cy - cf*sy, cf*st*cy + sf*sy;
         ct*sy, sf*st*sy + cf*cy, cf*st*sy - sf*cy;
         -st,   sf*ct,            cf*ct];
end
