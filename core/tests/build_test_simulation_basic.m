function test_simulation = build_test_simulation_basic( ...
    q0_principal, ...
    omega0_deg_s_principal ...
)
arguments
    q0_principal (4,1) double = [1;0;0;0]
    omega0_deg_s_principal  (3,1) double = [2;-1.5;3]
end

t0 = datetime(2024, 4, 17, 12, 0, 0);
tspan = 0:1:30;

moon_loc = planetEphemeris(juliandate(t0),'Earth','Moon')';
moon_loc_unit = moon_loc ./ norm(moon_loc);
moon_loc_perp = cross(moon_loc_unit,[0;0;1]);

x0_ECI = moon_loc - 100 .* moon_loc_unit;
v0_ECI = 6.48*moon_loc_perp;

% Body Frame Inertia Tensor
I_BC = [35.818 -2.5 3.0;-2.5 51.211 -5.5;3.0 -5.5 64.456];

surfaces = sc_surfaces();
sc_surface_area = 0;
for i=1:length(surfaces)
    sc_surface_area = sc_surface_area + surfaces(i).Area;
end

m_max = 4e-7*pi*sc_surface_area*0.1;
mx = (rand()-0.5)*2*m_max;
my = (rand()-0.5)*2*m_max;
mz = (rand()-0.5)*2*m_max;
magnetic_moment = [mx;my;mz];

initialState = SpacecraftState(x0_ECI,v0_ECI,[1;0;0;0],[0;0;0]);
spacecraft = Spacecraft(initialState,I_BC,magnetic_moment,surfaces);

%% Update initial state with the real omega0
omega0_deg_s_body = spacecraft.RPrincipalBody*omega0_deg_s_principal;
omega0_rad_s_body = deg2rad(omega0_deg_s_body);
spacecraft.InitialState.OmegaBody = omega0_rad_s_body;

%% Update initial state with the real q0
% q_body_principal = spacecraft.QPrincipalBody;
% q_inertial_body = quaternion(obj.QInertialBody);
% q_inertial_principal = quatmultiply(q_inertial_body,q_body_principal);

q0_body = quatmultiply(q0_principal',compact(spacecraft.QPrincipalBody));

% q0_body = quatmultiply(compact(quatinv(spacecraft.QPrincipalBody)),q0_principal');
compact(spacecraft.QPrincipalBody)
spacecraft.InitialState.QInertial = q0_body;

bodyNames = ["Earth","Moon","Sun"];

simulationConfig = SimulationConfig();
simulationConfig.GravityGradientTorqueEnabled = false;
simulationConfig.MagneticFieldTorqueEnabled = false;
simulationConfig.AerodynamicTorqueEnabled = false;
simulationConfig.SolarRadiationPressureTorqueEnabled = false;

test_simulation = Simulation(spacecraft,bodyNames,t0,tspan,simulationConfig);
end

