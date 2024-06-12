classdef SpacecraftState
    properties
        RInertial (3,1) double
        VInertial (3,1) double
        QInertial (4,1) double
        OmegaBody (3,1) double
        IInertial (3,3) double
        WheelOmegas (1,:) double
        OmegaMeasurement AngularVelocityMeasurement
        UnitVectorMeasurements (1,:) UnitVectorMeasurement
    end
    
    methods
        function obj = SpacecraftState( ...
            x_eci, ...
            v_eci, ...
            q_eci2b, ...
            omega_body, ...
            varargin ...
        )
            %SPACECRAFTSTATE Construct an instance of this class
            p = inputParser;
            addOptional(p,'wheel_omegas',[]);
            parse(p,varargin{:}); 
            obj.RInertial = x_eci;
            obj.VInertial = v_eci;
            obj.QInertial = q_eci2b;
            obj.OmegaBody = omega_body;
            obj.WheelOmegas = p.Results.wheel_omegas;
        end

        function obj = measure(obj, simulation_state)
            arguments
                obj SpacecraftState
                simulation_state SimulationState
            end
            spacecraft = simulation_state.Simulation.Spacecraft;
            obj.OmegaMeasurement = ...
                spacecraft.AngularVelocitySensor.measure(obj.OmegaBody);
            for i=1:length(spacecraft.UnitVectorSensors)
                obj.UnitVectorMeasurements(i) = ...
                    spacecraft.UnitVectorSensors(i).measure(simulation_state);
            end
        end

        function translational_state = translational_state_vec(obj)
            arguments
                obj SpacecraftState
            end
            translational_state = [obj.RInertial;obj.VInertial];
        end

        function att_state = attitude_state_vec(obj)
            arguments
                obj SpacecraftState
            end
            att_state = [obj.QInertial;obj.OmegaBody;obj.WheelOmegas'];
        end

        function full_state = full_state_vec(obj)
            arguments
                obj SpacecraftState
            end
            full_state = [obj.translational_state_vec();...
                          obj.attitude_state_vec()];
        end
    end
end

