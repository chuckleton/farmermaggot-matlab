function H = compute_sensitivity_matrix(r,A)
arguments (Input)
    r (3,1) double  % Inertial frame unit vector
    A (3,3) double  % Inertial -> Body attitude matrix
end
arguments (Output)
    H (3,6) double % Sensitivity matrix
end
    H = [cross_rep(A'*r) zeros(3,3)];
end

