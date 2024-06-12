function test_simulation = build_test_simulation_wheel( ...
    q0, ...
    omega0_deg_s, ...
    radius, ...
    ic_frame ...
)
arguments
    q0 (4,1) double = [1;0;0;0]
    omega0_deg_s  (3,1) double = [2;-1.5;3]
    radius (1,1) double = 100;
    ic_frame string = "principal"
end

t0 = datetime(2024, 4, 17, 12, 0, 0);
tspan_hours=0;
tspan_minutes=1;
tspan_seconds=0;
delta_t = 1;
tspan = 0:delta_t:tspan_hours*3600 + tspan_minutes*60 + tspan_seconds;

x0 = [radius;0;0];
mu = 4903;
v0 = [0;sqrt(mu/radius);0];

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

%% Sensors
bias_sun = [0.001;0;0];
bias_star = [0.00005;0;0];
sigma_sun = 0.003;
sigma_star = 0.00017;
% bias_sun = zeros(3,1);
% sigma_sun = 1e-5;
% bias_star = zeros(3,1);
% sigma_star = 1e-5;
sun_sensor = UnitVectorSensor(1,"Sun",bias_sun,sigma_sun);
star_tracker = UnitVectorSensor(1,"Star",bias_star,sigma_star);
unit_vector_sensors = [star_tracker sun_sensor];

bias_gyro = [0.01;-0.005;0.002];
% bias_gyro = zeros(3,1);
sigma_gyro = 1e-4;

angular_velocity_sensor = AngularVelocitySensor(1.0,bias_gyro,sigma_gyro);

initialState = SpacecraftState( ...
    x0,v0,[1;0;0;0],[0;0;0]);

%% Add reaction wheel(s)
% Square pyramid configuration
wheel_axes = (1/sqrt(3))*[-1 1 1 -1;-1 -1 1 1;1 1 1 1];
wheel_moment = 1e-3;

wheels = ReactionWheel.empty();
for i=1:size(wheel_axes,2)
    wheels(i) = ReactionWheel(wheel_axes(:,i),wheel_moment);

%% Create Spacecraft
spacecraft = Spacecraft(initialState,I_BC,magnetic_moment,surfaces, ...
    'unitVectorSensors',unit_vector_sensors, ...
    'angularVelocitySensor',angular_velocity_sensor, ...
    'reactionWheels',wheels);

%% Update initial state with the real omega0
if ic_frame == "principal"
    omega0_deg_s_body = spacecraft.RPrincipalBody*omega0_deg_s;
    omega0_rad_s_body = deg2rad(omega0_deg_s_body);
    spacecraft.InitialState.OmegaBody = omega0_rad_s_body;
elseif ic_frame == "body"
    spacecraft.InitialState.OmegaBody = deg2rad(omega0_deg_s);
end

%% Update initial state with the real q0
if ic_frame == "principal"
    q0_body = quatmultiply(q0',compact(spacecraft.QPrincipalBody));
    spacecraft.InitialState.QInertial = q0_body;
elseif ic_frame == "body"
    spacecraft.InitialState.QInertial = q0;
end
spacecraft.InitialState.EstimatedQInertial = spacecraft.InitialState.QInertial;
spacecraft.InitialState.WheelOmegas = zeros(length(wheels),1);

bodyNames = ["TestBody","Sun","Moon"];

simulationConfig = SimulationConfig();
simulationConfig.GravityGradientTorqueEnabled = true;
simulationConfig.MagneticFieldTorqueEnabled = false;
simulationConfig.AerodynamicTorqueEnabled = false;
simulationConfig.SolarRadiationPressureTorqueEnabled = false;

%% Create MEKF
P0 = [1e-2^2*eye(3) zeros(3,3);zeros(3,3) 0.01^2*eye(3)];
x0 = MEKFState( ...
    spacecraft.InitialState.QInertial,zeros(3,1),zeros(6,1),P0);
sigma_u = 5e-3;
sigma_v = 1e-5;
mekf = MEKF(x0,sigma_u,sigma_v,delta_t);

test_simulation = Simulation( ...
    spacecraft,bodyNames,mekf,t0,tspan,simulationConfig);
end

