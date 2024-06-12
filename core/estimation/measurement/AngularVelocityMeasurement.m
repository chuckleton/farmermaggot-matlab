classdef AngularVelocityMeasurement
    %ANGULARVELOCITYMEASUREMENT
    
    properties
        Sensor AngularVelocitySensor
        Omega (3,1) double
    end
    
    methods
        function obj = AngularVelocityMeasurement(sensor,omega)
            %ANGULARVELOCITYMEASUREMENT Construct an instance of this class
            arguments
                sensor AngularVelocitySensor
                omega (3,1) double
            end
            obj.Sensor = sensor;
            obj.Omega = omega;
        end
    end
end

