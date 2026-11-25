function omega = speedsFromInputs(mix, U)
%SPEEDSFROMINPUTS  Rotor speeds needed to produce U (4x1).
    omegaSq = mix.M \ (U(:)./mix.scale);
    if any(omegaSq <= 0)
        error(['The commanded inputs need a negative squared rotor speed; ', ...
               'the reference is too aggressive for these settings.']);
    end
    omega = sqrt(omegaSq);
end
