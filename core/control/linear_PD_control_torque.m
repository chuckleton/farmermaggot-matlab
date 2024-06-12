function L_bar = linear_PD_control_torque(state)
%LINEAR_PD_CONTROL_TORQUE
arguments (Input)
    state SimulationState
end
arguments (Output)
    L_bar (3,1) double 
end
    q_targ = state.ControlState.TargetQInertial;
    % q_actual = state.SpacecraftState.QInertial;
    q_actual = state.MekfIteration.XKPlus.QRef;
    delta_q = quaternion_error(q_targ',q_actual')';

    mu = 4903;
    radius = 1740+100;
    omega_orbit = sqrt(mu/radius^3);
    omega_c = [omega_orbit;0;0];

    omega = state.SpacecraftState.OmegaBody;

    dq4 = delta_q(1);
    dq13 = delta_q(2:4);

    kp = 50;
    kd = 50;

    L_bar = -kp*sign(dq4)*dq13 - kd*(omega-omega_c);
end

