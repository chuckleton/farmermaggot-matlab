function F_i_SRP = solar_radiation_force( ...
    simulationState, ...
    surface ...
)
arguments
    simulationState SimulationState
    surface Surface
end
r_spacecraft_eci = simulationState.SpacecraftState.RInertial;
q_spacecraft = simulationState.SpacecraftState.QInertial;

% Get the position vector from the Earth to the Sun
r_earth_sun = simulationState.BodyStates("Sun").Body.rEarthBody(simulationState.t);

% Get the position vector from the satellite to the sun
r_sat_sun = r_earth_sun - r_spacecraft_eci;
SRP = solar_radiation_pressure(r_sat_sun);

inclination = surface_inclination(quat2rotm(q_spacecraft')*surface.Normal,r_sat_sun);

F_i_SRP = -SRP*surface.Area*(2*...
    (surface.R_diff/3 + surface.R_spec*inclination)*surface.Normal ...
    + (1-surface.R_spec)*r_sat_sun/norm(r_sat_sun))*max(0,inclination);
end

