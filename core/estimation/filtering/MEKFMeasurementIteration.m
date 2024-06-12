classdef MEKFMeasurementIteration
    %MEKFMEASUREMENTITERATION
    
    properties
        Measurement UnitVectorMeasurement
        XIMinus MEKFState
        XIPlus MEKFState
        MeasurementResidualPre (3,1) double
        MeasurementResidualPostSingle (3,1) double
        MeasurementResidualPostOverall (3,1) double = zeros(3,1)
    end
    
    methods
        function obj = MEKFMeasurementIteration(measurement,x_i_minus, ...
                x_i_plus,residual_pre,residual_post_single)
            %MEKFMEASUREMENTITERATION Construct an instance of this class
            obj.Measurement = measurement;
            obj.XIMinus = x_i_minus;
            obj.XIPlus = x_i_plus;
            obj.MeasurementResidualPre = residual_pre;
            obj.MeasurementResidualPostSingle = residual_post_single;
        end
    end
end

