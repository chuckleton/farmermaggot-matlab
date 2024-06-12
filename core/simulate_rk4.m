function simulation_states = simulate_rk4(simulation)
%RK4
arguments
    simulation Simulation
end
    tspan = simulation.Tspan;
    dt = tspan(2) - tspan(1);

    simulation_states = SimulationState.empty;
    initial_sc_state = simulation.Spacecraft.InitialState;
    current_state = simulation_state_from_vec( ...
        simulation, ...
        tspan(1), ...
        initial_sc_state.full_state_vec() ...
     );
    simulation_states(1) = current_state;

    for i=1:length(tspan)-1
        t = tspan(i);
        next_state_vec = rk4(t,dt,current_state);
        next_state = simulation_state_from_vec( ...
            simulation,t,next_state_vec,'PreviousState',current_state);
        simulation_states(i+1) = next_state;
        current_state = next_state;
    end
end

