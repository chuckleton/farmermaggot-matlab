function tau_mf = magnetic_field_torque(simulationState)
% Gets magnetic field torque (ECI frame)
%INPUTS:
%   t: time to calculate at (datetime)
%   r_spacecraft_eci: 3x1 vector of spacecraft position in ECI frame
%   m: 3x1 vector of the spacecraft magnetic moment
%OUTPUTS:
%   tau_mf: 3x1 vector of magnetic field torque
arguments
    simulationState SimulationState
end
    t_seconds = simulationState.t;
    t0 = simulationState.Simulation.T0;
    t_datetime = t0 + seconds(t_seconds);

    spacecraft = simulationState.Simulation.Spacecraft;
    q_spacecraft = simulationState.SpacecraftState.QInertial;

    r_spacecraft_eci = simulationState.SpacecraftState.RInertial;

    % Convert r from ECI to LLA
    lla = eci2lla(r_spacecraft_eci',datevec(t_datetime));
    lat = lla(1);
    lon = lla(2);
    height = -lla(3);

    % Get the decimal year
    dy = decyear(t_datetime);

    % Get earth magnetic field (North-East-Down reference frame, nT)
    [B_ned_nT,~,~,~,~,~,~,~,~,~] = igrfmagm(height,lat,lon,dy);

    % Convert from nT to T
    B_ned = B_ned_nT * 1e-9;

    % Convert B from NED to ECEF
    [B_ecef_U,B_ecef_V,B_ecef_W] = ned2ecefv(B_ned(1),B_ned(2),B_ned(3),lat,lon);
    B_ecef = [B_ecef_U;B_ecef_V;B_ecef_W];

    % Convert B from ECEF to ECI (use velocity to get correct transform)
    [~,B_eci] = ecef2eci(datevec(t_datetime),zeros(3,1),B_ecef);

    % Compute torque M = mxB
    tau_mf = cross(quat2rotm(q_spacecraft')*spacecraft.MagneticMoment,B_eci);
end

