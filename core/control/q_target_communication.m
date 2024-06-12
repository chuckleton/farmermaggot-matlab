function q_target = q_target_communication(simulation_state)
%Q_TARGET_COMMUNICATION Compute target attitude for communication (anti-aligned to Earth RTN)
arguments
    simulation_state SimulationState
end
body_name = "Earth";
body_state = simulation_state.BodyStates(body_name);

q_target_zyx = body_state.QECI2RTN;

% Rotate to get x,y,-z alignment
% 1. Rotate -90 around y (note matlab rotations are passive)
q1 = rotm2quat(roty(-90));

q_target = quatmultiply(q1,q_target_zyx);
end

