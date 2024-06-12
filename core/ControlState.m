classdef ControlState
    %CONTROLSTATE
    
    properties
        TargetQInertial (4,1) double
        UnitVectorMeasurements (1,:) UnitVectorMeasurement
        EstimatedQInertial (4,1) double
        CommandTorque (3,1) double
        DesaturationTorque(3,1) double
        WheelTorques (1,:) double
        WheelsAppliedTorque (3,1) double
        AppliedTorque (3,1) double
        ThrusterPWPFState (1,:) double
        ThrusterCommandThrusts (1,:) double
        ThrusterAppliedThrusts (1,:) double
        ThrustersAppliedTorque (3,1) double
    end
    
    methods
        function obj = ControlState(simulation_state)
            %CONTROLSTATE Construct an instance of this class
            arguments
                simulation_state SimulationState
            end
            target_q_inertial = q_target_sensing(simulation_state);

            obj.TargetQInertial = target_q_inertial;

            spacecraft = simulation_state.Simulation.Spacecraft;
            unit_vector_measurements = UnitVectorMeasurement.empty;
            for unit_vector_sensor=spacecraft.UnitVectorSensors
                meas = unit_vector_sensor.measure(simulation_state);
                unit_vector_measurements(length(unit_vector_measurements)+1) = meas;
            end
            obj.UnitVectorMeasurements = unit_vector_measurements;

            % est_q_inertial = TRIAD(obj.UnitVectorMeasurements);
            est_q_inertial = q_method(obj.UnitVectorMeasurements);
            % est_q_inertial = simulation_state.EstimatedSpacecraftState.QInertial;
            obj.EstimatedQInertial = est_q_inertial;

            simulation_state.ControlState = obj;

            obj.CommandTorque = compute_control_torque(simulation_state);

            obj.DesaturationTorque = zeros(3,1);
            if simulation_state.DesaturationFlag
                L = spacecraft.compute_wheels_angular_momentum( ...
                    simulation_state.SpacecraftState.WheelOmegas);
                obj.DesaturationTorque = desaturation_torque(0.15,L);
            end
        end
    end
end

