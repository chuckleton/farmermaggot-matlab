classdef AngularVelocitySensor
    %ANGULARVELOCITYSENSOR
    
    properties
        Weight (1,1) double
        Bias (3,1) double
        Sigma (1,1) double
        Cov (3,3) double
    end
    
    methods
        function obj = AngularVelocitySensor(weight,bias,sigma)
            %ANGULARVELOCITYSENSOR Construct an instance of this class
            obj.Weight = weight;
            obj.Bias = bias;
            obj.Sigma = sigma;
            obj.Cov = sigma^2*eye(3);
        end

        function meas = measure(obj, true_omega, noisy)
            arguments (Input)
                obj AngularVelocitySensor
                true_omega (3,1) double
                noisy logical = true
            end
            arguments (Output)
                meas AngularVelocityMeasurement
            end
            meas = AngularVelocityMeasurement(obj,true_omega);

            % Add noise if needed
            if noisy
                meas = obj.addMeasurementNoise(meas);
            end
        end

        function noisy_meas = addMeasurementNoise(obj, meas)
            arguments (Input)
                obj AngularVelocitySensor
                meas AngularVelocityMeasurement
            end
            arguments (Output)
                noisy_meas AngularVelocityMeasurement
            end
            % Add noise
            noisy_meas = mvnrnd(meas.Omega',obj.Cov)' + obj.Bias;

            noisy_meas = AngularVelocityMeasurement( ...
                meas.Sensor,noisy_meas);
        end
    end
end

