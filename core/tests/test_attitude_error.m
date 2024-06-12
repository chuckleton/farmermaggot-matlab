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
% q1 = rotm2quat(eul2rotm([25 35 35]))
q0_b = quatmultiply(q0_b',q1);

% We rotate around x since we transformed
omega_0_rad_s_b = rad2deg([omega_orbit;0;0]);

simulation = build_test_simulation_circular(q0_b,omega_0_rad_s_b,radius,"body");
simulation = simulation.simulate();

% Convert q to euler angles
euler_angles_321 = rad2deg(quat2eul(simulation.Result.QInertialPrincipal));
% euler_angles_321_RTN = rad2deg(quat2eul(simulation.Result.QRTNPrincipal{"TestBody"}));
% euler_angles_321 = compact(simulation.Result.QInertialPrincipal);
% euler_angles_321 = simulation.Result.QRTNInertial{"TestBody"};
euler_angles_321_RTN = simulation.Result.QRTNPrincipal{"TestBody"};
omega_deg_s = simulation.Result.OmegaPrincipal;

t = simulation.Result.T;

target_q = simulation.Result.TargetQInertial;
actual_q = simulation.Result.QInertialBody;
error_q = simulation.Result.TargetActualErrorQ;

nplots = 3;

% Plot target attitude (Inertial)
subplot(nplots,1,1)
hold on
plot(t, target_q(:,1), 'DisplayName', 'q1')
plot(t, target_q(:,2), 'DisplayName', 'q2')
plot(t, target_q(:,3), 'DisplayName', 'q3')
plot(t, target_q(:,4), 'DisplayName', 'q4')
hold off
legend
xlabel("t (s)")
title("Target Attitude (ECI)")

subplot(nplots,1,2)
hold on
plot(t, actual_q(:,1), 'DisplayName', 'q1')
plot(t, actual_q(:,2), 'DisplayName', 'q2')
plot(t, actual_q(:,3), 'DisplayName', 'q3')
plot(t, actual_q(:,4), 'DisplayName', 'q4')
hold off
legend
xlabel("t (s)")
title("Actual Attitude (ECI)")

subplot(nplots,1,3)
hold on
plot(t, error_q(:,1), 'DisplayName', 'q1')
plot(t, error_q(:,2), 'DisplayName', 'q2')
plot(t, error_q(:,3), 'DisplayName', 'q3')
plot(t, error_q(:,4), 'DisplayName', 'q4')
hold off
legend
xlabel("t (s)")
title("Attitude Error")
