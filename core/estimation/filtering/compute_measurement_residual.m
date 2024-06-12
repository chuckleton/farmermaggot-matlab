function epsilon = compute_measurement_residual(b,r,A)
arguments (Input)
    b (3,1) double % Body unit vector
    r (3,1) double % Inertial unit vector
    A (3,3) double % Inertial -> Body attitude matrix
end
arguments (Output)
    epsilon (3,1) double % Measurement residual
end
    epsilon = b - A'*r;
end