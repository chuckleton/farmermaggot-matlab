function L_bar = linear_PD_control_omega_torque(state, omega_c)
%LINEAR_PD_CONTROL_TORQUE
arguments (Input)
    state SimulationState
    omega_c (3,1) double
end
arguments (Output)
    L_bar (3,1) double 
end
    omega = state.SpacecraftState.OmegaBody;
    kd = 50;

    L_bar = -kd*(omega-omega_c);
end

