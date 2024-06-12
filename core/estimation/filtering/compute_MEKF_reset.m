function mekf_state_reset = compute_MEKF_reset(mekf_state_unreset)
arguments (Input)
    mekf_state_unreset MEKFState
end
arguments (Output)
    mekf_state_reset MEKFState 
end
    q_minus = mekf_state_unreset.QRef;
    beta_minus = mekf_state_unreset.BetaRef;
    delta_x_minus = mekf_state_unreset.DeltaX;
    dtheta_minus = delta_x_minus(1:3);
    dbeta_minus = delta_x_minus(4:6);

    q_plus = compute_quaternion_reset(q_minus,dtheta_minus);
    Beta_plus = beta_minus + dbeta_minus;
    delta_x_plus = zeros(6,1);
    P_plus = mekf_state_unreset.P;

    mekf_state_reset = MEKFState(q_plus,Beta_plus,delta_x_plus,P_plus);
    % mekf_state_reset = MEKFState(q_minus,beta_minus,delta_x_plus,P_plus);
end

