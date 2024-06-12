function P_sun = solar_radiation_pressure(r_spacecraft_sun)
% Gets inclinations of surfaces relative to vec
%INPUTS:
%   r_spacecraft_sun: 3x1 vector from the spacecraft to the sun (m)
%OUTPUTS:
%   P: solar radiation pressure (N/m^2)
F = 1362;   % solar constant (W/m^2)
c = 299792458;  % speed of light (m/s)
r = norm(r_spacecraft_sun);

P_sun = F / (c*r);
end

