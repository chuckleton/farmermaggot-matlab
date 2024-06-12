classdef UnitVectorMeasurement
    %UNITVECTORMEASUREMENT
    
    properties
        Sensor UnitVectorSensor
        ReferenceAxis (3,1) double
        BodyAxis (3,1) double
    end
    
    methods
        function obj = UnitVectorMeasurement(sensor, reference_axis, body_axis)
            %UNITVECTORMEASUREMENT Construct an instance of this class
            arguments
                sensor UnitVectorSensor
                reference_axis (3,1) double
                body_axis (3,1) double
            end
            obj.Sensor = sensor;
            obj.ReferenceAxis = reference_axis;
            obj.BodyAxis = body_axis;
        end
    end
end

