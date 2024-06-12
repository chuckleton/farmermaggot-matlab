function obj = simulation_state_from_vec(simulation,t,vec,NameValueArgs)
    arguments (Input)
        simulation Simulation
        t double
        vec (:,1) double
        NameValueArgs.PreviousState = 0 
    end
    arguments (Output)
        obj SimulationState
    end
    % State is specified as: [r_eci,v_eci,q,omega,wheel_omegas]'
    x_ECI = vec(1:3);
    v_ECI = vec(4:6);
    q = vec(7:10);
    omega = vec(11:13);

    n_wheels = length(simulation.Spacecraft.ReactionWheels);
    wheel_omegas = zeros(1,0);
    if n_wheels > 0
        wheel_omegas = vec(14:14+n_wheels-1)';
    end

    % Create SpacecraftState
    spacecraft_state = SpacecraftState( ...
        x_ECI,v_ECI,q,omega,wheel_omegas);

    % Create SimulationState
    obj = SimulationState( ...
        simulation, ...
        t, ...
        spacecraft_state, ...
        'PreviousState',NameValueArgs.PreviousState);
end
