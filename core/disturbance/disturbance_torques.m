function tau_disturbance = disturbance_torques( ...
    simulationState ...
)
arguments
    simulationState SimulationState
end
    simulationConfig = simulationState.Simulation.Config;

    tau_gg_ECI = zeros(3,1);
    tau_mg_ECI = zeros(3,1);
    tau_srp_ECI = zeros(3,1);
    tau_aero_ECI = zeros(3,1);

    if simulationConfig.GravityGradientTorqueEnabled
        tau_gg_ECI = gravity_gradient_torque(simulationState);
    end

    if simulationConfig.MagneticFieldTorqueEnabled
        tau_mg_ECI = magnetic_field_torque(simulationState);
    end

    if simulationConfig.SolarRadiationPressureTorqueEnabled
        tau_srp_ECI = solar_radiation_pressure_torque(simulationState);
    end

    if simulationConfig.AerodynamicTorqueEnabled
        tau_aero_ECI = aero_torque(simulationState);
    end

    tau_disturbance = tau_gg_ECI + tau_mg_ECI + tau_srp_ECI + tau_aero_ECI;
end

