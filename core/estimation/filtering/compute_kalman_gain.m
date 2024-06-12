function K = compute_kalman_gain( ...
    P, ...
    H, ...
    sigma ...
)
arguments (Input)
    P (6,6) double
    H (3,6) double
    sigma (1,1) double
end
arguments (Output)
    K (6,3) double
end
    K = P*H'/(H*P*H' + sigma^2*eye(3));
end

