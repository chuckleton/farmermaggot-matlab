% Tests attitude error over time
clear;
close all;

mu = 4903;
radius = 1740+100;

T_orbit = 2*pi*sqrt(radius^3/mu);
omega_orbit = sqrt(mu/radius^3);

q0_b = [1;0;0;0];
% Rotate 90 around y 
q1 = rotm2quat(roty(-90));
q0_b = quatmultiply(q0_b',q1);

% We rotate around x since we transformed
omega_0_rad_s_b = rad2deg([omega_orbit;0;0]);

simulation = build_test_simulation_wheel(q0_b,omega_0_rad_s_b,radius,"body");
simulation = simulation.simulate();

t = simulation.Result.T;
t = t/60;

actual_q = simulation.Result.QInertialBody;
estimated_q = simulation.Result.EstimatedQInertial;
error_q = simulation.Result.EstimatedActualErrorQ;
error_q_axang = quat2axang(error_q);
error_ang = rad2deg(error_q_axang(:,4));

nplots = 3;
