% Tests attitude error over time
clear;
close all;
echo off;

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
t = t/60;

simulation.Result.OmegaInertial
simulation.Result.OmegaBody

actual_q = simulation.Result.QInertialBody;
estimated_q = simulation.Result.EstimatedQInertial;
error_q = simulation.Result.EstimatedActualErrorQ;
error_q_axang = quat2axang(error_q);
error_ang = rad2deg(error_q_axang(:,4));

nplots = 3;

% Plot estimated vs actual attitude (Inertial)
subplot(nplots,1,1)
hold on
plot(t, estimated_q(:,1), 'DisplayName', 'Estimated q1', 'LineStyle','--','Color',"#0072BD")
plot(t, estimated_q(:,2), 'DisplayName', 'Estimated q2', 'LineStyle','--','Color',"#D95319")
plot(t, estimated_q(:,3), 'DisplayName', 'Estimated q3', 'LineStyle','--','Color',"#EDB120")
plot(t, estimated_q(:,4), 'DisplayName', 'Estimated q4', 'LineStyle','--','Color',"#7E2F8E")
plot(t, actual_q(:,1), 'DisplayName', 'Actual q1','Color',"#0072BD")
plot(t, actual_q(:,2), 'DisplayName', 'Actual q2','Color',"#D95319")
plot(t, actual_q(:,3), 'DisplayName', 'Actual q3','Color',"#EDB120")
plot(t, actual_q(:,4), 'DisplayName', 'Actual q4','Color',"#7E2F8E")
hold off
legend
xlabel("t (minutes)")
title("Estimated vs Actual Quaternion")

subplot(nplots,1,2)
hold on
plot(t, error_q(:,1), 'DisplayName', 'q1')
plot(t, error_q(:,2), 'DisplayName', 'q2')
plot(t, error_q(:,3), 'DisplayName', 'q3')
plot(t, error_q(:,4), 'DisplayName', 'q4')
hold off
legend
xlabel("t (minutes)")
title("Attitude Error")

subplot(nplots,1,3)
hold on
plot(t, error_ang)
hold off
xlabel("t (minutes)")
ylabel("Error Angle Magnitude (deg)")
title("Attitude Error Angle Magnitude")

% %% Check small angle approx
% error_euler_approx = error_q(:,2:4)*2;
% 
% error_quat_approx = eul2quat(error_euler_approx,"XYZ");
% error_quat_diff = quaternion_error(error_q,error_quat_approx);
% 
% eqaa = quat2axang(error_quat_diff);
% ea = rad2deg(eqaa(:,4));
% 
% figure
% hold on
% plot(t, ea)
% hold off
% xlabel("t (minutes)")
% ylabel("Error Angle Magnitude (deg)")
% title("Small Angle Approximation Error")

%% Check covariance
cov = simulation.Result.AttitudeCovariance;

cov_diag = zeros(size(cov,1),3);
for i=1:size(cov,1)
    cov_diag(i,:) = diag(squeeze(cov(i,:,:)));
end

std = sqrt(cov_diag);
error_q_rod = quat2rod(error_q);
ci_low = -1.96*std;
ci_high = 1.96*std;

figure
hold on
plot(t,error_q_rod);
plot(t,ci_low,'LineStyle','--')
plot(t,ci_high,'LineStyle','--')
hold off
mycolors = [1 0 0; 0 1 0; 0 0 1];
ax = gca; 
ax.ColorOrder = mycolors;
xlabel("t (minutes)")
ylabel("Rodrigues Parameter")
title("MEKF Rodrigues Error Parameters and Covariance")

% std_axang = quat2axang(rod2quat(std));
% std_ang = std_axang(:,4);
% 
% figure
% hold on
% plot(t, error_ang, 'DisplayName', "Actual error angle", 'LineWidth', 2)
% plot(t, std_ang, 'DisplayName', "1\sigma covariance angle", 'LineWidth', 2)
% hold off
% xlabel("t (minutes)")
% ylabel("Error Angle Magnitude (deg)")
% title("True Error Angle Magnitude vs EKF Covariance")
% legend

%% Measurement Residuals
pre_resids = simulation.Result.MeasurementPreResiduals;
post_single_resids = simulation.Result.MeasurementPostSingleResiduals;
post_overall_resids = simulation.Result.MeasurementPostOverallResiduals;

pre_norm = squeeze(vecnorm(pre_resids,2,3));
post_single_norm = squeeze(vecnorm(post_single_resids,2,3));
post_overall_norm = squeeze(vecnorm(post_overall_resids,2,3));

% take the last 10 minutes of post_overall_norms for statistics
avg_post_overall_norm = mean(post_overall_norm(end-60*10:end,:),1);
std_post_overall_norm = std(post_overall_norm(end-60*10:end,:),0,1);

figure
plot(t,pre_norm,'LineStyle','--','Color','red','DisplayName',"Original");
hold on;
% plot(t,post_single_norm,'LineStyle',':','DisplayName',"Post Single Update");
plot(t,post_overall_norm,'LineStyle','-','DisplayName',"Post Overall Update");
hold off;
xlabel("t (minutes)")
ylabel("Residual Magnitude")
title("Measurement Residuals Before & After Measurement Update")
legend

%% Estimated Gyro Bias
est_beta = simulation.Result.EstimatedBeta;
bias_gyro = [0.01;-0.005;0.002];

beta_cov = simulation.Result.BetaCovariance;
beta_std = zeros(size(beta_cov,1),3);
for i=1:size(beta_cov,1)
    beta_std(i,:) = sqrt(diag(squeeze(beta_cov(i,:,:))));
end
ci_low = est_beta - 1.96*beta_std;
ci_high = est_beta + 1.96*beta_std;

figure
plot(t,est_beta);
hold on;
yline(bias_gyro);
plot(t,ci_low,'LineStyle','--')
plot(t,ci_high,'LineStyle','--')
hold off
mycolors = [1 0 0; 0 1 0; 0 0 1];
ax = gca; 
ax.ColorOrder = mycolors;
xlabel("t (minutes)")
ylabel("Estimated Gyro Bias (rad/s)")
title("MEKF Estimated Gyro Bias Convergence")