function q_k_plus_1 = propagate_quaternion_first_order(q_k,omega_k,delta_t)
arguments (Input)
    q_k (4,1) double
    omega_k (3,1) double
    delta_t (1,1) double
end
arguments (Output)
    q_k_plus_1 (4,1) double
end

psi = sin(0.5*norm(omega_k)*delta_t)*omega_k / norm(omega_k);

Theta_11 = cos(0.5*norm(omega_k)*delta_t)*eye(3) - cross_rep(psi);
Theta_12 = psi;
Theta_21 = -psi';
Theta_22 = cos(0.5*norm(omega_k)*delta_t);

 % Change indices for scalar-first quaternion
Theta = [Theta_22 Theta_21;Theta_12 Theta_11];

q_k_plus_1 = Theta * q_k;
end

