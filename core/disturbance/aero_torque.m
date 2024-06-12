function tau_aero = aero_torque(simulationState)
arguments
    simulationState SimulationState
end
    surfaces = simulationState.Simulation.Spacecraft.Surfaces;

    tau_aero = zeros(3,1);
    for i=1:length(surfaces)
        surface = surfaces(i);
        f_aero = aero_force(simulationState,surface);
        tau_aero = tau_aero + cross(surface.Barycenter,f_aero);
    end
end

