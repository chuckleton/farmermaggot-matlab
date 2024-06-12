function F_i_aero = aero_force( ...
    simulationState, ...
    surface ...
)
arguments
    simulationState SimulationState
    surface Surface
end

t0 = simulationState.Simulation.T0;
t = simulationState.t;
r_spacecraft_eci = simulationState.SpacecraftState.RInertial;
v_spacecraft_eci = simulationState.SpacecraftState.VInertial;
omega_spacecraft_eci = quatrotate(...
    quatinv(simulationState.SpacecraftState.QInertial'), ...
    simulationState.SpacecraftState.OmegaBody' ...
)';

rho = rho_atmosphere(t,t0,r_spacecraft_eci);

omega_earth = 0.000072921158553;    % Earth angular velocity (rad/s)
omega_earth_vec = omega_earth*[0;0;1];
v_rel_CM = v_spacecraft_eci + cross(omega_earth_vec,r_spacecraft_eci); % km/s
v_rel_CM = v_rel_CM*1e3;
v_rel_i = v_rel_CM + cross(omega_spacecraft_eci,surface.Barycenter);

inclination = surface_inclination(surface.Normal,v_rel_i);
F_i_aero = -0.5*rho*surface.Cd*surface.Area*norm(v_rel_i)*v_rel_i*max(0,inclination);
end

