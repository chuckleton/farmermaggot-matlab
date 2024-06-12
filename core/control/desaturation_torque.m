function desat_torque = desaturation_torque( ...
    tau_d,L)
%DESATURATION_TORQUES
    eps = 0.03;
    L_prime = zeros(size(L));
    for i=1:length(L)
        if abs(L(i)) > eps
            L_prime(i) = L(i);
        end
    end
    desat_torque = tau_d * sign(L_prime) / sqrt(3);
end

