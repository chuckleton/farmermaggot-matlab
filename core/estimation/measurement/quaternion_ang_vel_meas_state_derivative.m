function [state_derivative] = quaternion_ang_vel_meas_state_derivative( ...
    t, ...
    state, ...
    omega, ...
    K)
% Gets the state derivative in quaternion attitude representation
%INPUTS:
%   t: time (s)
%   state: [q,omega,omega_wheels]' (quaternion real part first)
%   spacecraft: Spacecraft object
%   K: scalar to encourage unit quaternion (defaults to 1.0)
%OUTPUTS:
%   state_derivative: Derivative of state
    arguments
        t double
        state (4,1) double
        omega (3,1) double
        K double = 1.0
    end

    % Extract state components
    q = state(1:4);

    cross_rep = @(v)[ 
        0      -v(3)    v(2)
        v(3)    0      -v(1)
        -v(2)    v(1)    0
        ];  % Cross-product representation of v

    % https://link.springer.com/content/pdf/10.1007/978-1-4939-0802-8.pdf
    % eq 3.20
    Q = [-q(2:4)';q(1)*eye(3)+cross_rep(q(2:4))];
    
    % Parameter to maintain unit quaternion
    epsilon = 1 - norm(q)^2;
    
    % Compute derivatives
    q_dot = 0.5 * Q * omega + K*epsilon*q;
    
    % Concatenate result
    state_derivative = q_dot;
end

