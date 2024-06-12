classdef MEKFIteration
    %MEKFITERATION
    
    properties
        XKMinus1Plus MEKFState
        XKMinus MEKFState
        MeasurementIterations (1,:) MEKFMeasurementIteration
        XKPlus MEKFState
        Omega (3,1) double
        Measurements (1,:) UnitVectorMeasurement
        Q (6,6) double
        DeltaT (1,1) double
    end
    
    methods
        function obj = MEKFIteration( ...
                x_k_minus_1_plus, ...
                omega, ...
                measurements, ...
                Q, ...
                delta_t ...
            )
            obj.XKMinus1Plus = x_k_minus_1_plus;
            obj.Omega = omega;
            obj.Measurements = measurements;
            obj.Q = Q;
            obj.DeltaT = delta_t;
        end
        
        function obj = propagate(obj)
            obj.XKMinus = propagate_MEKF( ...
                obj.XKMinus1Plus, ...
                obj.Omega-obj.XKMinus1Plus.BetaRef, ...
                obj.Q, ...
                obj.DeltaT ...
            );
        end

        function obj = measurementUpdate(obj)
            prev_state = obj.XKMinus;
            A_inertial_body = quat2rotm(obj.XKMinus.QRef');
            for i=1:size(obj.Measurements,2)
                measurement = obj.Measurements(i);
                mekf_iteration = compute_single_measurement_iteration( ...
                    prev_state,measurement,A_inertial_body);
                obj.MeasurementIterations(i) = mekf_iteration;
                prev_state = mekf_iteration.XIPlus;
            end
        end

        function obj = reset(obj)
            last_measurement_iteration = obj.MeasurementIterations( ...
                length(obj.MeasurementIterations));
            post_reset_state = compute_MEKF_reset( ...
                last_measurement_iteration.XIPlus);
            obj.XKPlus = post_reset_state;

            A_plus = quat2rotm(post_reset_state.QRef');

            % Compute the measurement residuals after the update
            for i=1:size(obj.Measurements,2)
                measurement = obj.Measurements(i);
                post_update_residual = compute_measurement_residual( ...
                    measurement.BodyAxis,measurement.ReferenceAxis,A_plus);
                obj.MeasurementIterations(i).MeasurementResidualPostOverall = ...
                    post_update_residual;
            end
        end
    end
end

