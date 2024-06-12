function state_dot = omega_state_derivative( ...
    t, state, spacecraft, wheel_torques, body_torques)
% Gets state derivative given current state
%INPUTS:
%   t: time to evaluate at (s)
%   state: current state: [omega, omega_wheels...]
%       where omega_wheels is each wheel angular velocity about its axis
%   I: 3x3 inertia tensor of the spacecraft (without reaction wheels)
%   I_wheels: nx2 matrix of inertia properties of n reaction wheels
%       for n=l, I_wheels(:,l) = [I_parallel, I_perpendicular]' to spin axis
%OUTPUTS:
%   state_dot: [omega_dot, omega_wheels_dot]
    arguments
        t double
        state (:,1) double
        spacecraft Spacecraft
        wheel_torques (1,:) double = zeros(1,0)
        body_torques (3,1) double = zeros(3,1)
    end

    %% Extract named parameters from inputs
    body_omega = state(1:3);
    wheel_omegas = state(4:end);

    %% Get properties from spacecraft
    I = spacecraft.IBody;
    wheel_axes = spacecraft.wheelAxes();
    wheel_Is = spacecraft.wheelIs();
    
    %% Compute angular momentum vector, torque vector, derivative of reaction wheels
    wheel_angular_momentums = zeros(3,size(wheel_Is,1));
    wheel_torque_vectors = zeros(3,size(wheel_omegas,1));
    wheel_omega_dots = zeros(size(wheel_omegas,1),1);
    for l = 1:size(wheel_omegas,1)
        % Get variables for current wheel
        wheel_I_parallel = wheel_Is(1,l);
        wheel_axis = wheel_axes(:,l);
        wheel_omega = wheel_omegas(l);
    
        % Compute angular momentum (vector)
        wheel_angular_momentum = wheel_I_parallel*( ...
            dot(wheel_axis,body_omega) + wheel_omega)*wheel_axis;
        wheel_angular_momentums(:,l) = wheel_angular_momentum;

        % Compute torque (vector)
        wheel_torque = wheel_torques(l);
        wheel_torque_vector = wheel_axis .* wheel_torque;
        wheel_torque_vectors(:,l) = wheel_torque_vector;
    end
    
    overall_wheel_angular_momentum = sum(wheel_angular_momentums,2);
    overall_wheel_torque_vector = sum(wheel_torque_vectors,2);
    
    %% Compute Ib
    % Ib is the inertia tensor of the spacecraft without wheels,
    % plus the components of the wheel inertia tensors perpendicular to spin
    wheel_perp_inertia_tensors = zeros(3,3,size(wheel_omegas,1));
    for l = 1:size(wheel_omegas,1)
        wheel_I_perpendicular = wheel_Is(2,l);
        wheel_axis = wheel_axes(:,l);
    
        wheel_perp_inertia_tensor = wheel_I_perpendicular*( ...
            eye(3) - wheel_axis*wheel_axis');
        wheel_perp_inertia_tensors(:,:,l) = wheel_perp_inertia_tensor;
    end
    overall_wheel_perp_inertia_tensor = sum(wheel_perp_inertia_tensors,3);
    Ib = I + overall_wheel_perp_inertia_tensor;
    
    %% omega_dot = Euler equations
    omega_dot = Ib \ (body_torques - overall_wheel_torque_vector - cross( ...
        body_omega, Ib*body_omega + overall_wheel_angular_momentum));
   
    for l = 1:size(wheel_omegas,1)
        % Get variables for current wheel
        wheel_I_parallel = wheel_Is(1,l);
        wheel_axis = wheel_axes(:,l);
        wheel_torque = wheel_torques(l);
    
        % Compute derivative (scalar)
        wheel_omega_dot = (wheel_torque - ...
            wheel_I_parallel*dot(wheel_axis,omega_dot)) / wheel_I_parallel;
        wheel_omega_dots(l,1) = wheel_omega_dot;
    end

    state_dot = [omega_dot;wheel_omega_dots];
end