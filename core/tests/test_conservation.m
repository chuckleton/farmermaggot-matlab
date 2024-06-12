% Tests conservation of rotational KE and angular momentum in inertial
% frame
clear;
close all;

simulation = build_test_simulation_basic();

simulation = simulation.simulate();

L = simulation.Result.LInertial;
E = simulation.Result.TInertial;

% Plot inertial angular momentum (Should be constant)
figure;
plot(L);

% Plot rotational kinetic energy (Should be constant)
figure;
plot(E);

% Plot polhode and angular momentum vector (perpendicular)
omega_inertial = simulation.Result.OmegaInertial;
figure
hold on;
plot3(omega_inertial(:,1),omega_inertial(:,2),omega_inertial(:,3), ...
    'LineWidth',2, ...
    'DisplayName', 'Polhode')
quiver3(0,0,0, ...
    L(1,1)*0.02, ...
    L(1,2)*0.02, ...
    L(1,3)*0.02, ...
    'DisplayName','Angular Momentum Vector')
legend;
axis equal
title('Polhode and Angular Momentum Vector')
xlabel('\omega_x (rad/s)')
ylabel('\omega_y (rad/s)')
zlabel('\omega_z (rad/s)')
hold off