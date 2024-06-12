function shadowed = is_in_moon_shadow(simulationState)
arguments
    simulationState SimulationState
end
    % Get vector from earth to sun
    % Get the position vector from the Earth to the Sun
    r_earth_sun = simulationState.BodyStates("Sun").Body.rEarthBody(simulationState.t);
    % Get vector from earth to moon
    r_earth_moon = simulationState.BodyStates("Moon").Body.rEarthBody(simulationState.t);
    r_moon_sat = simulationState.SpacecraftState.RInertial - r_earth_moon;
    r_moon_sun = r_earth_sun - r_earth_moon;
    R_moon = 1737;  % Moon radius (km)
    
    % Check for shadowing (cylindrical shadow model)
    if dot(r_moon_sat,r_moon_sun) / r_moon_sun < -sqrt(norm(r_moon_sat)^2-R_moon^2)
        shadowed = 1;
    else
        shadowed = 0;
    end
end

