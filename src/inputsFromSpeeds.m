function U = inputsFromSpeeds(mix, omega)
%INPUTSFROMSPEEDS  Thrust and torques produced by four rotor speeds (4x1 in, 4x1 out).
    U = mix.scale .* (mix.M*(omega(:).^2));
end
