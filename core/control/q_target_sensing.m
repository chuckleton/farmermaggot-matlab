function q_target = q_target_sensing(simulation_state)
%Q_TARGET_SENSING Compute target attitude for sensing (aligned to RTN)
arguments
    simulation_state SimulationState
end
body_name = "TestBody";
body_state = simulation_state.BodyStates(body_name);

q_target_zyx = body_state.QECI2RTN';

% Rotate to get xyz -> +n,+t,-r alignment
% 1. Rotate -90 around y
q1 = rotm2quat(roty(-90));

q_target = quatmultiply(q_target_zyx,q1);
end

