clear;
close all;

r1 = [0.61;0.38;0.46];
r2 = [0.61;-0.38;0.46];
r3 = [0.61;-0.38;-0.46];
r4 = [0.61;0.38;-0.46];

e1 = [1;0.5;0.5];
e2 = [1;-0.5;0.5];
e3 = [1;-0.5;-0.5];
e4 = [1;0.5;-0.5];

e1 = e1/norm(e1);
e2 = e2/norm(e2);
e3 = e3/norm(e3);
e4 = e4/norm(e4);

A = [cross_rep(r1)*e1 cross_rep(r2)*e2 cross_rep(r3)*e3 cross_rep(r4)*e4];