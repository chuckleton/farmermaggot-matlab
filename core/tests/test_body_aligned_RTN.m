% Tests that simulation works properly with principal axes aligned to RTN
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

simulation = build_test_simulation_circular(q0_b,omega_0_rad_s_b,radius,"body");
simulation = simulation.simulate();

t = simulation.Result.T;
t = t/60; % Convert to minutes

nplots = 3;

% Plot
subplot(nplots,1,1)
target_q = simulation.Result.TargetQInertial;
actual_q = simulation.Result.QInertialBody;
hold on
plot(t, target_q(:,1), 'DisplayName', 'Target q1', 'LineStyle','--','Color',"#0072BD")
plot(t, target_q(:,2), 'DisplayName', 'Target q2', 'LineStyle','--','Color',"#D95319")
plot(t, target_q(:,3), 'DisplayName', 'Target q3', 'LineStyle','--','Color',"#EDB120")
plot(t, target_q(:,4), 'DisplayName', 'Target q4', 'LineStyle','--','Color',"#7E2F8E")
plot(t, actual_q(:,1), 'DisplayName', 'Actual q1','Color',"#0072BD")
plot(t, actual_q(:,2), 'DisplayName', 'Actual q2','Color',"#D95319")
plot(t, actual_q(:,3), 'DisplayName', 'Actual q3','Color',"#EDB120")
plot(t, actual_q(:,4), 'DisplayName', 'Actual q4','Color',"#7E2F8E")
hold off
legend
xlabel("t (minutes)")
title("Target vs Actual Quaternion")

subplot(nplots,1,2)
error_q = simulation.Result.TargetActualErrorQ;
error_q_axang = quat2axang(error_q);
error_ang = abs(rad2deg(error_q_axang(:,4)));
hold on
plot(t, error_q(:,1), 'DisplayName', '\deltaq1')
plot(t, error_q(:,2), 'DisplayName', '\deltaq2')
plot(t, error_q(:,3), 'DisplayName', '\deltaq3')
plot(t, error_q(:,4), 'DisplayName', '\deltaq4')
hold off
legend
xlabel("t (minutes)")
title("Attitude Error Quaternion")

subplot(nplots,1,3)
hold on
plot(t, error_ang)
hold off
xlabel("t (minutes)")
ylabel("Error Angle Magnitude (deg)")
title("Attitude Error Angle Magnitude")