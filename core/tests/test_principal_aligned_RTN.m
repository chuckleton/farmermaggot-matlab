% Tests that simulation works properly with principal axes aligned to RTN
clear;
close all;

mu = 4903;
radius = 1740+100;

T_orbit = 2*pi*sqrt(radius^3/mu);
omega_orbit = sqrt(mu/radius^3);

omega_0_rad_s_p = rad2deg([0;0.0*omega_orbit;omega_orbit]);

q0_p = [1;0;0;0];

simulation = build_test_simulation_circular(q0_p,omega_0_rad_s_p,radius);
simulation = simulation.simulate();

% Convert q to euler angles
euler_angles_321 = rad2deg(quat2eul(simulation.Result.QInertialPrincipal));
euler_angles_321_RTN = rad2deg(quat2eul(simulation.Result.QRTNPrincipal{"TestBody"}));
omega_deg_s = simulation.Result.OmegaPrincipal;
t = simulation.Result.T;

% Plot angular velocities and angles
subplot(4,1,1)
hold on
plot(t, euler_angles_321(:,1), 'DisplayName', 'Yaw')
plot(t, euler_angles_321(:,2), 'DisplayName', 'Pitch')
plot(t, euler_angles_321(:,3), 'DisplayName', 'Roll')
hold off
legend
xlabel("t (s)")
ylabel("Euler Angle (deg)")

subplot(4,1,2)
hold on
plot(t, omega_deg_s(:,3), 'DisplayName', 'Yaw Rate (rad/s)')
plot(t, omega_deg_s(:,2), 'DisplayName', 'Pitch Rate (rad/s)')
plot(t, omega_deg_s(:,1), 'DisplayName', 'Roll Rate (rad/s)')
hold off
legend
xlabel("t (s)")
ylabel("Angular Rate (deg/s)")

subplot(4,1,3)
hold on
plot(t, euler_angles_321_RTN(:,1), 'DisplayName', 'Yaw')
plot(t, euler_angles_321_RTN(:,2), 'DisplayName', 'Pitch')
plot(t, euler_angles_321_RTN(:,3), 'DisplayName', 'Roll')
hold off
legend
xlabel("t (s)")
ylabel("Euler Angle (deg)")

subplot(4,1,4)
tau_ggs = simulation.Result.DisturbanceTorques;
hold on
plot(t, tau_ggs(:,1), 'DisplayName', 'x')
plot(t, tau_ggs(:,2), 'DisplayName', 'y')
plot(t, tau_ggs(:,3), 'DisplayName', 'z')
hold off
legend
xlabel("t (s)")
ylabel("Euler Angle (deg)")