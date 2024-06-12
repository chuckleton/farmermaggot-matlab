clear;
close all;

axes = ['x','y','z'];

%% Problem 1a
omega_0_mag_deg_s = 3; % deg/s
euler_angles_321_0_principal = deg2rad([0 0 0]);
% q_0_principal = [1;0;0;0];
q_0_principal = eul2quat(euler_angles_321_0_principal);

figure
for constant_axis = 1:3
    % Initial conditions (pure rotation about constant_axis)
    omega0_deg_s_principal = zeros(3,1);
    omega0_deg_s_principal(constant_axis,1) = omega_0_mag_deg_s;
    
    % Numerically simulate
    simulation = build_test_simulation_basic(q_0_principal,omega0_deg_s_principal);
    simulation = simulation.simulate();
    
    % Convert q to euler angles
    euler_angles_321 = rad2deg(quat2eul(simulation.Result.QInertialPrincipal));
    omega_deg_s = rad2deg(simulation.Result.OmegaPrincipal);
    t = simulation.Result.T;
    
    % Plot angular velocities and angles
    subplot(2,3,constant_axis)
    hold on
    plot(t, euler_angles_321(:,1), 'DisplayName', 'Yaw')
    plot(t, euler_angles_321(:,2), 'DisplayName', 'Pitch')
    plot(t, euler_angles_321(:,3), 'DisplayName', 'Roll')
    hold off
    legend
    title(sprintf("Pure rotation about %s (%d deg/s)", ...
        axes(constant_axis),omega_0_mag_deg_s))
    xlabel("t (s)")
    ylabel("Euler Angle (deg)")
    
    subplot(2,3,constant_axis+3)
    hold on
    plot(t, omega_deg_s(:,3), 'DisplayName', 'Yaw Rate (rad/s)')
    plot(t, omega_deg_s(:,2), 'DisplayName', 'Pitch Rate (rad/s)')
    plot(t, omega_deg_s(:,1), 'DisplayName', 'Roll Rate (rad/s)')
    hold off
    legend
    xlabel("t (s)")
    ylabel("Angular Rate (deg/s)")
end