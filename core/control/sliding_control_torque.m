function L_bar = sliding_control_torque(state)
%SLIDING_CONTROL_TORQUE
arguments (Input)
    state SimulationState
end
arguments (Output)
    L_bar (3,1) double 
end
    J = state.Simulation.Spacecraft.IBody;
    q_targ = state.ControlState.TargetQInertial;
    % q_actual = state.SpacecraftState.QInertial;
    q_actual = state.MekfIteration.XKPlus.QRef;
    delta_q = quaternion_error(q_targ',q_actual')';

    omega = state.SpacecraftState.OmegaBody;

    mu = 4903;
    radius = 1740+100;
    omega_orbit = sqrt(mu/radius^3);
    omega_c = [omega_orbit;0;0];

    dq4 = delta_q(1);
    dq13 = delta_q(2:4);

    k = 1;
    G = 1*eye(3);
    epsilon = 1;

    s = (omega - omega_c) + k*sign(dq4)*dq13;
    s_bar = saturation(s,epsilon);

    L_bar = J*( ...
        (k/2)*( ...
            abs(dq4)*(omega_c-omega) - ...
            sign(dq4)*cross(dq13,omega_c+omega)) - G*s_bar) + ...
        cross_rep(omega)*J*omega;
end

