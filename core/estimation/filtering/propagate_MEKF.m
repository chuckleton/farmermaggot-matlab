function mekf_state_k_minus = ...
    propagate_MEKF(mekf_state_k_minus_1_plus, omega, Q, delta_t)
arguments (Input)
    mekf_state_k_minus_1_plus (1,1) MEKFState
    omega (3,1) double
    Q (6,6) double
    delta_t (1,1) double
end
arguments (Output)
    mekf_state_k_minus (1,1) MEKFState
end
    q_k_minus_1_plus = mekf_state_k_minus_1_plus.QRef;
    P_k_minus_1_plus = mekf_state_k_minus_1_plus.P;
    
    %% Propagate quaternion
    q_k_minus = propagate_quaternion_first_order( ...
        q_k_minus_1_plus,omega,delta_t);
    
    %% Propagate gyro biases
    beta_k_minus = mekf_state_k_minus_1_plus.BetaRef;
    
    %% Propagate covariance
    P_k_minus = propagate_covariance_MEKF( ...
        P_k_minus_1_plus,omega,Q,delta_t);
    
    mekf_state_k_minus = MEKFState(q_k_minus,beta_k_minus,zeros(6,1),P_k_minus);
end

