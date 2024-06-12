function state_dot = state_derivative(t,state,simulationState)
%STATE_DERIVATIVE
arguments
    t double
    state (:,1) double
    simulationState SimulationState
end
    % simulationState = simulation_state_from_vec(simulation,t,state);
    simulation = simulationState.Simulation;

    %% Translation
    translational_state = state(1:6);
    translational_derivative = n_body_state_derivative(...
        t,translational_state,simulation);

    %% Compute External Torques
    disturbance_body_torques_ECI = disturbance_torques(simulationState);
    %% Convert body torques from ECI to body
    disturbance_body_torques = quatrotate( ...
        state(7:10)', ...
        disturbance_body_torques_ECI')';

    %% Compute thruster torques
    thruster_torques = simulationState.ControlState.ThrustersAppliedTorque;

    body_torques = disturbance_body_torques + thruster_torques;

    %% Compute Control
    wheel_torques = simulationState.ControlState.WheelTorques;

    %% Rotation
    attitude_state = state(7:end);
    attitude_derivative = quaternion_state_derivative(t,attitude_state, ...
        simulation.Spacecraft, ...
        wheel_torques,body_torques);

    %% Combined
    state_dot = [translational_derivative;...
                 attitude_derivative];
end

