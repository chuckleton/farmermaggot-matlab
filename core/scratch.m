clear;

s = tf('s');

K_m = 1;
tau_m = 1;
num = K_m;
den = [tau_m 1];

L = tf(num,den);

step(L);


