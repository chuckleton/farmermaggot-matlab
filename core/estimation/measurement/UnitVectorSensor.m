classdef UnitVectorSensor
    %UNITVECTORSENSOR A sensor that returns a unit vector
    
    properties
        Weight (1,1) double
        Type string
        Bias (3,1) double
        Sigma (1,1) double
        Cov (3,3) double
    end
    
    methods
        function obj = UnitVectorSensor(weight, type, bias, sigma)
            %UNITVECTORSENSOR Construct an instance of this class
            obj.Weight = weight;
            obj.Type = type;
            obj.Bias = bias;
            obj.Sigma = sigma;
            obj.Cov = sigma^2*eye(3);
        end

        function meas = measure(obj, simulation_state, noisy)
            arguments (Input)
                obj UnitVectorSensor
                simulation_state SimulationState
                noisy logical = true
            end
            arguments (Output)
                meas UnitVectorMeasurement
            end

            % Get the raw measurement
            if obj.Type == "Sun"
                meas = obj.measureSunSensor(simulation_state);
            elseif obj.Type == "Star"
                meas = obj.measureStarTracker(simulation_state);
            end
            
            % Add noise
            if noisy
                meas = obj.addMeasurementNoise(meas);
            end
        end

        function noisy_meas = addMeasurementNoise(obj, meas)
            arguments (Input)
                obj UnitVectorSensor
                meas UnitVectorMeasurement
            end
            arguments (Output)
                noisy_meas UnitVectorMeasurement
            end
            % Add noise
            noisy_meas_body = mvnrnd(meas.BodyAxis',obj.Cov)' + obj.Bias;
            % Normalize
            noisy_meas_body = noisy_meas_body / norm(noisy_meas_body);

            noisy_meas = UnitVectorMeasurement( ...
                meas.Sensor,meas.ReferenceAxis,noisy_meas_body);
        end

        function meas = measureSunSensor(obj, simulation_state)
            arguments
                obj UnitVectorSensor
                simulation_state SimulationState
            end
            % Get the unit vector from the spacecraft to the sun
            r_earth_sun = simulation_state.BodyStates("Sun").Body.rEarthBody(simulation_state.t);
            r_earth_sat = simulation_state.SpacecraftState.RInertial;
            r_sat_sun_ref = r_earth_sun - r_earth_sat;
            r_sat_sun_ref_unit = r_sat_sun_ref / norm(r_sat_sun_ref);

            q_eci_body = simulation_state.SpacecraftState.QInertial;
            r_sat_sun_body_unit = quatrotate(q_eci_body',r_sat_sun_ref_unit')';

            meas = UnitVectorMeasurement(obj,r_sat_sun_ref_unit,r_sat_sun_body_unit);
        end

        function meas = measureStarTracker(obj, simulation_state)
            arguments
                obj UnitVectorSensor
                simulation_state SimulationState
            end
            % Get the unit vector from the spacecraft to the star
            % (approx constant)
            r_sat_star_ref = [1;2;3] / norm([1;2;3]);   % Some random orientation
            r_sat_star_ref_unit = r_sat_star_ref / norm(r_sat_star_ref);

            q_eci_body = simulation_state.SpacecraftState.QInertial;
            r_sat_star_body_unit = quatrotate(q_eci_body',r_sat_star_ref_unit')';

            meas = UnitVectorMeasurement(obj,r_sat_star_ref_unit,r_sat_star_body_unit);
        end
    end
end

