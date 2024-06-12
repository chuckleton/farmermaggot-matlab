function new_measurement_iteration = compute_single_measurement_iteration( ...
    x_i_minus, ...
    measurement, ...
    A_inertial_body ...
)
arguments (Input)
    x_i_minus MEKFState
    measurement UnitVectorMeasurement
    A_inertial_body (3,3) double
end
arguments (Output)
    new_measurement_iteration MEKFMeasurementIteration
end
    q_minus = x_i_minus.QRef;
    Beta_minus = x_i_minus.BetaRef;
    P_minus = x_i_minus.P;
    delta_x_minus = x_i_minus.DeltaX;
    
    r = measurement.ReferenceAxis;
    b = measurement.BodyAxis;
    sigma = measurement.Sensor.Sigma;
    
    H = compute_sensitivity_matrix(r,A_inertial_body);
    K = compute_kalman_gain(P_minus,H,sigma);
    P_plus = compute_covariance_update(P_minus,K,H);
    epsilon = compute_measurement_residual(b,r,A_inertial_body);
    delta_x_plus = delta_x_minus + K*(epsilon-H*delta_x_minus);

    q_post = compute_quaternion_reset(q_minus,delta_x_plus(1:3));
    A_inertial_body_post = quat2rotm(q_post');
    epsilon_post = compute_measurement_residual(b,r,A_inertial_body_post);
    
    x_i_plus = MEKFState(q_minus,Beta_minus,delta_x_plus,P_plus);
    
    new_measurement_iteration = MEKFMeasurementIteration( ...
        measurement,x_i_minus,x_i_plus,epsilon,epsilon_post);
end

