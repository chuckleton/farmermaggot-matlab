function q_plus = compute_quaternion_reset( ...
    q_minus, ...
    dtheta_plus ...
)
arguments
    q_minus (4,1) double % Previous attitude quaternion
    dtheta_plus (3,1) double % Updated state error estimate
end
    xi_q = xi_rep(q_minus);
    innovation = 0.5*xi_q*dtheta_plus;
    q_plus_unnorm = q_minus + innovation;
    q_plus = q_plus_unnorm / norm(q_plus_unnorm);
end