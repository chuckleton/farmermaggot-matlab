function L_dot_w = compute_wheel_torques(A,A_star,M_c,omega,L_w)
%COMPUTE_WHEEL_TORQUES Summary of this function goes here
arguments (Input)
    A (:,3) double % Wheel mounting matrix
    A_star (3,:) double % Wheel mounting matrix pseudoinverse
    M_c (3,1) double % Commanded actuation torque
    omega (3,1) double % Angular velocity
    L_w (:,1) double % Wheel angular momentum
end
arguments (Output)
    L_dot_w (:,1) double
end
    L_dot_w = -A_star * (M_c + cross(omega,A*L_w));
end

