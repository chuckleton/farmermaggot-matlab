function L_bar = compute_control_torque(state)
%COMPUTE_CONTROL_TORQUE
arguments (Input)
    state SimulationState
end
arguments (Output)
    L_bar (3,1) double 
end
    controller_type = state.Simulation.Config.ControllerType;
    if controller_type == "sliding"
        L_bar = sliding_control_torque(state);
    elseif controller_type == "linear_PD"
        L_bar = linear_PD_control_torque(state);
    elseif controller_type == "nonlinear_PD"
        L_bar = nonlinear_PD_control_torque(state);
    else
        error("Unknown controller_type")
    end
end

