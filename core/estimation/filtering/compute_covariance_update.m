function P_plus = compute_covariance_update(P_minus,K,H)
arguments (Input)
    P_minus (6,6) double % Previous covariance matrix
    K (6,3) double % Kalman gain matrix
    H (3,6) double % Sensitivity matrix
end
arguments (Output)
    P_plus (6,6) double % Updated covariance matrix
end
    P_plus = (eye(6)-K*H)*P_minus;
end