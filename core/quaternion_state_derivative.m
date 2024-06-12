function [state_derivative] = quaternion_state_derivative( ...
    t, ...
    state, ...
    spacecraft, ...
    wheel_torques, ...
    body_torques, ...
    NameValueArgs ...
)
% Gets the state derivative in quaternion attitude representation
%INPUTS:
%   t: time (s)
%   state: [q,omega,omega_wheels]' (quaternion real part first)
%   spacecraft: Spacecraft object
%   wheel_torques: 1xm vector of wheel torques
%   body_torques: 3x1 vector of body torques
%   K: scalar to encourage unit quaternion (defaults to 1.0)
%OUTPUTS:
%   state_derivative: Derivative of state
    arguments (Input)
        t double
        state (:,1) double
        spacecraft Spacecraft
        wheel_torques (1,:) double = zeros(1,0)
        body_torques (3,1) double = zeros(3,1)
        NameValueArgs.K double = 1.0
        NameValueArgs.OmegaNoise (3,1) double = [0;0;0]
    end
    K = NameValueArgs.K;

    % Extract state components
    q = state(1:4);
    omega = state(5:end);

    % Add noise to omega (if needed)
    omega(1:3) = omega(1:3) + NameValueArgs.OmegaNoise;
    omega_body = omega(1:3);

    % https://link.springer.com/content/pdf/10.1007/978-1-4939-0802-8.pdf
    % eq 3.20
    Q = [-q(2:4)';q(1)*eye(3)+cross_rep(q(2:4))];
    
    % Parameter to maintain unit quaternion
    epsilon = 1 - norm(q)^2;
    
    % Compute derivatives
    q_dot = 0.5 * Q * omega_body + K*epsilon*q;
    omega_dot = omega_state_derivative(t,omega,spacecraft, ...
        wheel_torques,body_torques);
    
    % Concatenate result
    state_derivative = [q_dot;omega_dot];
end

