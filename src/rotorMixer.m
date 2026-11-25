function mix = rotorMixer(cfg)
%ROTORMIXER  Linear map between squared rotor speeds and the inputs U1..U4.
%   [U1;U2;U3;U4] ./ scale = M * omega.^2
    mix.M = [ 1  1  1  1;
              0  1  0 -1;
             -1  0  1  0;
             -1  1 -1  1];
    l = cfg.armLength;
    mix.scale = [cfg.thrustCoeff; cfg.thrustCoeff*l; cfg.thrustCoeff*l; cfg.torqueCoeff];
end
