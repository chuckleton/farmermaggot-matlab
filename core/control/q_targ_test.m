
q_target_zyx = eul2quat(deg2rad([180 -90 0]))

% Rotate to get x,-y,z alignment
% 1. Rotate 180 around x
q1 = rotm2quat(rotx(180));
% 2. Rotate 90 around y (note matlab rotations are passive)
q2 = rotm2quat(roty(-90));

q_target = quatmultiply(q2,quatmultiply(q1,q_target_zyx))
eul_target = rad2deg(quat2eul(q_target))