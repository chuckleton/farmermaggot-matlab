% Tests that Polhode is on Momentum and Energy Ellipsoids in principal axes
clear;
close all;

simulation = build_test_simulation_basic();

simulation = simulation.simulate();

L_ellipsoid = momentum_ellipsoid(simulation.Spacecraft.IPrincipal, ...
    simulation.Result.LPrincipal(1,:));
E_ellipsoid = energy_ellipsoid(simulation.Spacecraft.IPrincipal, ...
    simulation.Result.TPrincipal(1));

L_ellipsoid.SemiAxes;
E_ellipsoid.SemiAxes;

figure;
hold on;
L_ellipsoid.plot('blue',0.5,'Momentum Ellipsoid')
E_ellipsoid.plot('red',0.5,'Energy Ellipsoid')
plot3(simulation.Result.OmegaPrincipal(:,1), ...
    simulation.Result.OmegaPrincipal(:,2), ...
    simulation.Result.OmegaPrincipal(:,3), ...
    'LineWidth',4, ...
    'Color','green',...
    'DisplayName', 'Polhode')
hold off;