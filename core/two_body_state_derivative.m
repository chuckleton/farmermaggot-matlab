function state_derivative = two_body_state_derivative(t,state)
%TWO_BODY_STATE_DERIVATIVE Summary of this function goes here
%   Detailed explanation goes here
% state: [x,y,z,vx,vy,vz] in ECEF frame
mu = 4903; % [km^3/sec^2]
r = state(1:3);
v = state(4:6);

r_unit = r ./ norm(r);
a = -r_unit .* mu / norm(r).^2;

r_dot = v;
v_dot = a;

state_derivative = [r_dot;v_dot];
end

