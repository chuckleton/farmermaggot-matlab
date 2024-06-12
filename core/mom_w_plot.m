clear;
close all;

A = (1/sqrt(3))*[-1 1 1 -1;-1 -1 1 1;1 1 1 1];
A_star = pinv(A);

t = 0:0.1:15;

tau_x = sin(t);
tau_y = cos(t);
tau_z = sin(t/2);

tau = 0.16*[tau_x;tau_y;tau_z];

L_w = A_star*tau;

L_w_noisy = awgn(L_w,50);


subplot(2,1,1)
hold on
plot(t,L_w_noisy(1,:),'DisplayName',"L_{w,1}");
plot(t,L_w_noisy(2,:),'DisplayName',"L_{w,2}");
plot(t,L_w_noisy(3,:),'DisplayName',"L_{w,3}");
plot(t,L_w_noisy(4,:),'DisplayName',"L_{w,4}");
hold off
legend
title("Applied Wheel Torques")
xlabel("t (s)")
ylabel("L_w (Nm)")

subplot(2,1,2)
hold on
plot(t,tau(1,:),'DisplayName',"L_1");
plot(t,tau(2,:),'DisplayName',"L_2");
plot(t,tau(3,:),'DisplayName',"L_3");
hold off
legend
title("Commanded Torques")
xlabel("t (s)")
ylabel("\tau (Nm)")