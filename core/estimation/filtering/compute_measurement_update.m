function [q_plus, Beta_plus, P_plus] = compute_measurement_update( ...
    q_minus, ...
    P_minus, ...
    Beta_minus ...
)

% Compute attitude matrix from q_minus
A_inertial_body = quat2rotm(q_minus);

% Initialize
delta_x_plus = zeros(6,1);
P_plus = P_minus;

for i=1:size(Beta_minus,1)
    r = Beta_minus(i,1);
    b = Beta_minus(i,2);
    sigma = Beta_minus(i,3);
    H = compute_sensitivity_matrix(r,A_inertial_body);
    K = compute_kalman_gain(P_minus,H,sigma);
    P_plus = compute_covariance_update(P_plus,K,H);
    epsilon = compute_measurement_residual(b,r,A);
    delta_x_plus = delta_x_plus + K*(epsilon-H*delta_x_plus);
end

dtheta_plus = delta_x_plus(1:3);
q_plus = compute_quaternion_reset(q_minus,dtheta_plus);
Beta_plus = beta_minus + delta_x_plus(4:6);
end

