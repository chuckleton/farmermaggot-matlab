function rho = rho_atmosphere( ...
    t, ...
    t0, ...
    r_spacecraft_eci ...
)
% Gets atmospheric density at a location (using NRLMSISE-00 model)
%INPUTS:
%   t: time to calculate at (datetime, UTC)
%   r_spacecraft_eci: 3x1 vector of spacecraft position in ECI frame (km)
%OUTPUTS:
%   rho: atmosphere density (kg/m^3)
% Convert r from ECI to LLA
arguments
    t double
    t0 datetime
    r_spacecraft_eci (3,1) double
end
    t_datetime = t0 + seconds(t);

    lla = eci2lla(1e3*r_spacecraft_eci',datevec(t_datetime));
    lat = lla(1);
    lon = lla(2);
    altitude = lla(3);
    

    [~,densities] = atmosnrlmsise00(altitude,lat,lon, ...
        year(t_datetime),day(t_datetime,'dayofyear'),second(t_datetime) ...
        );
    % Extract the 6th element of densities (total mass density, kg/m^3)
    rho = densities(6);
end

