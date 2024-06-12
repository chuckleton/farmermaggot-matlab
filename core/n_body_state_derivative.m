function state_derivative = n_body_state_derivative( ...
    t, ...
    state, ...
    simulation ...
)
% t: time (s) since t0
% state: [x,y,z,vx,vy,vz] in ECEF frame
% simulation: Simulation object
arguments
    t double
    state (6,1) double
    simulation Simulation
end
r_sat_earth = state(1:3);
v = state(4:6);

a_total = zeros(3,1);
for body = simulation.Bodies
    mu = body.Mu;
    r_earth_body = body.rEarthBody(t);
    r_sat_body = r_sat_earth - r_earth_body;
    r_unit_body = r_sat_body / norm(r_sat_body);
    a_body = -r_unit_body * mu / norm(r_sat_body)^2;
    a_total = a_total + a_body;
end

r_dot = v;
v_dot = a_total;

state_derivative = [r_dot;v_dot];
end