function P_k_plus_1_minus = propagate_covariance_MEKF( ...
    P_k_plus, omega_k, Q, delta_t)
arguments (Input)
    P_k_plus (6,6) double
    omega_k (3,1) double
    Q (6,6) double
    delta_t (1,1) double
end
arguments (Output)
    P_k_plus_1_minus (6,6) double
end
dt = delta_t;
w = norm(omega_k);
wa = cross_rep(omega_k);

%% Compute Phi
Phi_11 = eye(3) + sin(w*dt)/w*wa + (1-cos(w*delta_t))/w^2*wa*wa;
Phi_12 = -(eye(3)*dt+wa*(1-cos(w*dt))/w^2+wa*wa*(w*dt-sin(w*dt))/w^3);
Phi = [Phi_11 Phi_12;zeros(3) eye(3)];

P_k_plus_1_minus = Phi*P_k_plus*Phi' + Q;
end

