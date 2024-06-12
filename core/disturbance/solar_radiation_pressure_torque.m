function tau_srp = solar_radiation_pressure_torque(simulationState)
arguments
    simulationState SimulationState
end
    % Check for shadowing, if so, early return
    if is_in_moon_shadow(simulationState)
        tau_srp = 0;
        return;
    end

    surfaces = simulationState.Simulation.Spacecraft.Surfaces;

    tau_srp = zeros(3,1);
    for i=1:length(surfaces)
        surface = surfaces(i);
        srp_force = solar_radiation_force(simulationState,surface);
        tau_srp = tau_srp + cross(surface.Barycenter,srp_force);
    end
end

