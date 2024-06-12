classdef SimulationState    
    properties
        Simulation Simulation
        t double
        SpacecraftState SpacecraftState
        BodyStates
        EstimatedBodyStates
        ControlState ControlState
        MekfIteration MEKFIteration
        DesaturationFlag logical
        DesaturationComplete logical;
    end
    
    methods
        function obj = SimulationState( ...
                simulation,t,spacecraftState,NameValueArgs)
            arguments
                simulation Simulation
                t double
                spacecraftState SpacecraftState
                NameValueArgs.PreviousState = 0
            end

            %% Body States
            bodyStates = dictionary();
            for body=simulation.Bodies
                bodyStates(body.Name) = BodyState( ...
                    body,t,simulation,spacecraftState);
            end
            obj.BodyStates = bodyStates;

            obj.Simulation = simulation;
            obj.t = t;
            obj.SpacecraftState = spacecraftState;

            %% Measure
            obj.SpacecraftState = obj.SpacecraftState.measure(obj);

            %% Desaturation
            if t > 150
                if NameValueArgs.PreviousState.DesaturationComplete
                    obj.DesaturationFlag = false;
                    obj.DesaturationComplete = true;
                else
                    L = ...
                        simulation.Spacecraft.compute_wheels_angular_momentum( ...
                        obj.SpacecraftState.WheelOmegas);
                    if norm(L) < 0.03*sqrt(3)
                        obj.DesaturationFlag = false;
                        obj.DesaturationComplete = true;
                    else
                        obj.DesaturationFlag = true;
                        obj.DesaturationComplete = false;
                    end
                end
            else
                obj.DesaturationFlag = false;
                obj.DesaturationComplete = false;
            end

            %% MEKF
            if ~isa(NameValueArgs.PreviousState,'double')
                obj.MekfIteration = simulation.Mekf.iterate( ...
                    NameValueArgs.PreviousState.MekfIteration, ...
                    obj.SpacecraftState.OmegaMeasurement.Omega, ...
                    obj.SpacecraftState.UnitVectorMeasurements);
            else
                obj.MekfIteration = MEKFIteration( ...
                    simulation.Mekf.InitialState, ...
                    obj.SpacecraftState.OmegaMeasurement.Omega, ...
                    obj.SpacecraftState.UnitVectorMeasurements, ...
                    simulation.Mekf.Q, ...
                    0);
                obj.MekfIteration.XKPlus = simulation.Mekf.InitialState;
            end

            %% Control
            control_state = ControlState(obj);
            obj.ControlState = control_state;


            % thruster_target_torque = -control_state.DesaturationTorque;
            thruster_target_torque = control_state.CommandTorque;
            desired_thruster_thrusts = ...
                    simulation.Spacecraft.compute_thruster_thrusts( ...
                    thruster_target_torque);
            if ~isa(NameValueArgs.PreviousState,'double')
                [thruster_commands, new_ufilt] = simulation.Pwpf.run( ...
                    NameValueArgs.PreviousState.ControlState.ThrusterPWPFState, ...
                    desired_thruster_thrusts, ...
                    NameValueArgs.PreviousState.ControlState.ThrusterCommandThrusts);
            else
                [thruster_commands, new_ufilt] = simulation.Pwpf.run( ...
                    zeros(size(desired_thruster_thrusts)), ...
                    desired_thruster_thrusts, ...
                    zeros(size(desired_thruster_thrusts)));
            end

            obj.ControlState.ThrusterPWPFState = new_ufilt;
            obj.ControlState.ThrusterCommandThrusts = thruster_commands;
            thruster_command_torque = ...
                simulation.Spacecraft.compute_body_torque_thrusters( ...
                thruster_commands);
            obj.ControlState.ThrusterAppliedThrusts = ...
                simulation.Spacecraft.add_thruster_command_noise( ...
                thruster_commands);
            obj.ControlState.ThrustersAppliedTorque = ...
                simulation.Spacecraft.compute_body_torque_thrusters( ...
                obj.ControlState.ThrusterAppliedThrusts);

            if obj.DesaturationFlag
                wheel_target_torque = -thruster_command_torque + ...
                    control_state.CommandTorque;
                % wheel_target_torque = -thruster_target_torque + control_state.CommandTorque;
            else
                % wheel_target_torque = control_state.CommandTorque;
                wheel_target_torque = zeros(3,1);
            end
            wheel_torques = simulation.Spacecraft.compute_wheel_torques( ...
                wheel_target_torque);
            wheels_applied_torque = simulation.Spacecraft.compute_body_torque_wheels( ...
                wheel_torques);

            obj.ControlState.WheelTorques = wheel_torques;
            obj.ControlState.WheelsAppliedTorque = wheels_applied_torque;

            obj.ControlState.AppliedTorque = ...
                obj.ControlState.WheelsAppliedTorque + ...
                obj.ControlState.ThrustersAppliedTorque;
        end
    end
end

