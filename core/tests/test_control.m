% Tests attitude error over time
clear;
close all;

mu = 4903;
radius = 1740+100;

T_orbit = 2*pi*sqrt(radius^3/mu);
omega_orbit = sqrt(mu/radius^3);

q0_b = [1;0;0;0];
% Rotate 90 around y
q1 = rotm2quat(roty(-92.5));
q0_b = quatmultiply(q0_b',q1);

% We rotate around x since we transformed
omega_0_rad_s_b = rad2deg([0;0;0]);
omega_target = [omega_orbit;0;0];

simulation = build_test_simulation_circular(q0_b,omega_0_rad_s_b,radius,"body");
simulation = simulation.simulate();

t = simulation.Result.T;

actual_q = simulation.Result.QInertialBody;
target_q = simulation.Result.TargetQInertial;
error_q = simulation.Result.TargetActualErrorQ;
error_q_axang = quat2axang(error_q);
error_ang = rad2deg(error_q_axang(:,4));

omega = simulation.Result.OmegaBody;
omega_error = rad2deg(omega - omega_target');

nplots = 4;

subplot(nplots,1,1)
hold on
plot(t, error_q(:,1), 'DisplayName', 'q1')
plot(t, error_q(:,2), 'DisplayName', 'q2')
plot(t, error_q(:,3), 'DisplayName', 'q3')
plot(t, error_q(:,4), 'DisplayName', 'q4')
hold off
legend
xlabel("t (s)")
title("Attitude Error")

subplot(nplots,1,2)
hold on
plot(t, error_ang)
hold off
xlabel("t (s)")
ylabel("Angle(deg)")
title("Attitude Error Angle Magnitude")

subplot(nplots,1,3)
hold on
plot(t,omega_error(:,1),'DisplayName','\delta\omega_x');
plot(t,omega_error(:,2),'DisplayName','\delta\omega_y');
plot(t,omega_error(:,3),'DisplayName','\delta\omega_z');
hold off
xlabel("t (s)")
ylabel("\delta\omega (deg/s)")
title("Angular Velocity Error")
legend

command_torques = simulation.Result.CommandTorques;
applied_torques = simulation.Result.AppliedTorques;

% Plot command torques
subplot(nplots,1,4)
hold on
C = orderedcolors("gem");
stairs(t,command_torques(:,1),'DisplayName',"Command \tau_x",'LineStyle','--');
stairs(t,command_torques(:,2),'DisplayName',"Command \tau_y",'LineStyle','--');
stairs(t,command_torques(:,3),'DisplayName',"Command \tau_z",'LineStyle','--');
stairs(t,applied_torques(:,1),'DisplayName',"Applied \tau_x");
stairs(t,applied_torques(:,2),'DisplayName',"Applied \tau_y");
stairs(t,applied_torques(:,3),'DisplayName',"Applied \tau_z");
colororder(C(1:3,:))
hold off
xlabel("t (s)")
ylabel("Torque (Nm)")
title("Commanded and Applied Control Torques")
legend


%% Plot Wheel States
figure
% nplots = 6;
nx = 3;
ny = 1;
% wheel_torques = simulation.Result.WheelTorques;
% omega_wheels = rad2deg(simulation.Result.OmegaWheels);
% wheel_applied_torques = simulation.Result.WheelsAppliedTorques;
% 
% subplot(nx,ny,1)
% hold on
% plot(t,wheel_torques(:,1),'DisplayName','\tau_1');
% plot(t,wheel_torques(:,2),'DisplayName','\tau_2');
% plot(t,wheel_torques(:,3),'DisplayName','\tau_3');
% plot(t,wheel_torques(:,4),'DisplayName','\tau_4');
% hold off
% xlabel("t (s)")
% ylabel("\tau (Nm)")
% title("Wheel Torque")
% legend
% 
% subplot(nx,ny,3)
% hold on
% plot(t,omega_wheels(:,1),'DisplayName','\omega_1');
% plot(t,omega_wheels(:,2),'DisplayName','\omega_2');
% plot(t,omega_wheels(:,3),'DisplayName','\omega_3');
% plot(t,omega_wheels(:,4),'DisplayName','\omega_4');
% hold off
% xlabel("t (s)")
% ylabel("\omega (deg/s)")
% title("Wheel Angular Velocity")
% legend
% 
% subplot(nx,ny,5)
% hold on
% stairs(t,wheel_applied_torques(:,1),'DisplayName','\tau_1');
% stairs(t,wheel_applied_torques(:,2),'DisplayName','\tau_2');
% stairs(t,wheel_applied_torques(:,3),'DisplayName','\tau_3');
% hold off
% xlabel("t (s)")
% ylabel("\tau [Nm]")
% title("Wheels Applied Body Torques")
% legend

%% Plot Thruster States
% figure
thruster_pwpf_states = simulation.Result.ThrusterPWPFStates;
thruster_thrusts = simulation.Result.ThrusterAppliedThrusts*4;
thruster_torques = simulation.Result.ThrustersAppliedTorques;

subplot(nx,ny,1)
hold on
C = orderedcolors("gem");
stairs(t,thruster_thrusts(:,1),'DisplayName','T_1');
stairs(t,thruster_thrusts(:,2),'DisplayName','T_2');
stairs(t,thruster_thrusts(:,3),'DisplayName','T_3');
stairs(t,thruster_thrusts(:,4),'DisplayName','T_4');
colororder(C(1:4,:))
hold off
xlabel("t (s)")
ylabel("T (N)")
title("Thruster Thrusts")
legend

subplot(nx,ny,2)
hold on
C = orderedcolors("gem");
plot(t,thruster_pwpf_states(:,1),'DisplayName','PWPF_1');
plot(t,thruster_pwpf_states(:,2),'DisplayName','PWPF_2');
plot(t,thruster_pwpf_states(:,3),'DisplayName','PWPF_3');
plot(t,thruster_pwpf_states(:,4),'DisplayName','PWPF_4');
colororder(C(1:4,:))
hold off
xlabel("t (s)")
ylabel("T (N)")
title("Thruster PWPF States")
legend

subplot(nx,ny,3)
hold on
stairs(t,thruster_torques(:,1),'DisplayName','\tau_1');
stairs(t,thruster_torques(:,2),'DisplayName','\tau_2');
stairs(t,thruster_torques(:,3),'DisplayName','\tau_3');
hold off
xlabel("t (s)")
ylabel("\tau [Nm]")
title("Thrusters Applied Body Torques")
legend