classdef Spacecraft
    %SPACECRAFT

    properties
        InitialState SpacecraftState % Spacecraft initial State
        ABody (3,3) double  % Body axes (default eye(3))
        IBody (3,3) double  % 3x3 inertia tensor in body axes
        IPrincipal (3,3) double     % 3x3 inertia tensor in principal axes
        APrincipal (3,3) double     % Principal axes
        RPrincipalBody (3,3) double    % Rotation matrix from body to principal axes
        QPrincipalBody quaternion % Quaternion from body to principal axes
        MagneticMoment (3,1) double     % Spacecraft magnetic moment
        Surfaces (1,:) Surface  % Spacecraft surfaces
        ReactionWheels (1,:) ReactionWheel  % Spacecraft reaction wheels
        ReactionWheelMountingMatrix double % Reaction Wheel Mounting Matrix
        ReactionWheelMountingMatrixStar double % Reaction Wheel Mounting Matrix
        Thrusters (1,:) Thruster % Spacecraft thrusters
        ThrusterMountingMatrix double % Thruster Mounting Matrix
        ThrusterMountingMatrixStar double % Thruster Mounting Matrix
        UnitVectorSensors (1,:) UnitVectorSensor    % Spacecraft unit vector sensors
        AngularVelocitySensor AngularVelocitySensor    % Spacecraft angular velocity sensor
    end
    
    methods
        function obj = Spacecraft(initialState, IBody, magneticMoment, surfaces, varargin)
            %SPACECRAFT Construct an instance of this class
            %INPUTS:
            %   initialState: Spacecraft initial state
            %   IBody: Spacecraft inertia tensor in body axes
            %   ABody: Body axes (optional, default eye(3))
            %OUTPUTS:
            %   obj: spacecraft object
         
            % Parse optional args
            p = inputParser;
            valid3x3matrix = @(x) isnumeric(x) && ismatrix(x) && isequal(size(x),[3,3]);
            defaultABody = eye(3);
            defaultReactionWheels = ReactionWheel.empty;
            defaultThrusters = Thruster.empty;
            defaultUnitVectorSensors = UnitVectorSensor.empty;
            defaultAngularVelocitySensors = AngularVelocitySensor.empty;
            addRequired(p,'initialState')
            addRequired(p,'IBody',valid3x3matrix);
            addRequired(p,'magneticMoment');
            addRequired(p,'surfaces');
            addParameter(p,'ABody',defaultABody,valid3x3matrix);
            addParameter(p,'reactionWheels',defaultReactionWheels);
            addParameter(p,'thrusters',defaultThrusters)
            addParameter(p,'unitVectorSensors',defaultUnitVectorSensors);
            addParameter(p,'angularVelocitySensor',defaultAngularVelocitySensors);
            parse(p,initialState,IBody,magneticMoment,surfaces,varargin{:});

            % Copy in InitialState
            obj.InitialState = p.Results.initialState;

            % Copy in IBody and ABody
            obj.IBody = p.Results.IBody;
            obj.ABody = p.Results.ABody;

            obj.MagneticMoment = p.Results.magneticMoment;

            % Copy in Surfaces
            obj.Surfaces = p.Results.surfaces;

            % Copy in ReactionWheels
            obj.ReactionWheels = p.Results.reactionWheels;
            obj.ReactionWheelMountingMatrix = obj.computeWheelMountingMatrix();
            obj.ReactionWheelMountingMatrixStar = pinv(obj.ReactionWheelMountingMatrix);

            % Copy in Thrusters
            obj.Thrusters = p.Results.thrusters;
            obj.ThrusterMountingMatrix = obj.computeThrusterMountingMatrix();
            obj.ThrusterMountingMatrixStar = pinv(obj.ThrusterMountingMatrix);

            % Copy in sensors
            obj.UnitVectorSensors = p.Results.unitVectorSensors;
            obj.AngularVelocitySensor = p.Results.angularVelocitySensor;

            % Compute principal inertia tensor
            % Body Axes
            [obj.IPrincipal, obj.APrincipal, obj.RPrincipalBody] = ...
                inertia_tensor_to_principal(IBody, obj.ABody);
            obj.QPrincipalBody = rotm2quat(obj.RPrincipalBody');
        end

        function A = computeWheelMountingMatrix(obj)
            arguments (Input)
                obj Spacecraft
            end
            wheels = obj.ReactionWheels;
            A = zeros(3,length(wheels));
            for i=1:length(wheels)
                wheel = wheels(i);
                A(:,i) = wheel.Axis;
            end
        end

        function wheel_axes = wheelAxes(obj)
            arguments
                obj Spacecraft
            end
            % Returns 3xn_wheels matrix of wheel axes
            wheel_axes = obj.ReactionWheelMountingMatrix;
        end

        function wheel_Is = wheelIs(obj)
            arguments
                obj Spacecraft
            end
            % Returns 2xn_wheels matrix of wheel axes
            wheel_Is = zeros(2,length(obj.ReactionWheels));
            for i=1:length(obj.ReactionWheels)
                reactionWheel = obj.ReactionWheels(i);
                wheel_Is(:,i) = reactionWheel.I;
            end
        end

        function wheel_torques = compute_wheel_torques(obj, command_torque)
            arguments
                obj Spacecraft
                command_torque (3,1) double 
            end
            wheel_torques = ...
                obj.ReactionWheelMountingMatrixStar * (-command_torque);
            % Make sure we don't exceed max allowed torque on wheels
            % And add random gaussian noise
            for i=1:length(wheel_torques)
                max_torque = obj.ReactionWheels(i).MaxTorque;
                sigma = obj.ReactionWheels(i).SigmaCommand;
                wheel_torques(i) = mvnrnd(clip( ...
                    wheel_torques(i),-max_torque,max_torque),sigma^2);
            end
        end

        function A = computeThrusterMountingMatrix(obj)
            arguments (Input)
                obj Spacecraft
            end
            thrusters = obj.Thrusters;
            A = zeros(3,length(thrusters));
            for i=1:length(thrusters)
                thruster = thrusters(i);
                A(:,i) = cross_rep(thruster.R)*thruster.E*thruster.T;
            end
        end

        function thruster_thrusts = compute_thruster_thrusts( ...
                obj, command_torque, noisy)
            arguments
                obj Spacecraft
                command_torque (3,1) double 
                noisy logical = false
            end
            thruster_thrusts = obj.ThrusterMountingMatrixStar * (-command_torque);

            if noisy
                for i=1:length(thruster_thrusts)
                    sigma = obj.Thrusters(i).SigmaCommand;
                    thruster_thrusts(i) = mvnrnd( ...
                        thruster_thrusts(i),sigma^2);
                end
            end
        end

        function thruster_thrusts = add_thruster_command_noise( ...
                obj, command_thrusts)
            arguments
                obj Spacecraft
                command_thrusts (1,:) double
            end
            thruster_thrusts = zeros(size(command_thrusts));
            for i=1:length(thruster_thrusts)
                    if command_thrusts(i) ~= 0
                        sigma = obj.Thrusters(i).SigmaCommand;
                        thruster_thrusts(i) = mvnrnd( ...
                            command_thrusts(i),sigma^2);
                    else
                        thruster_thrusts(i) = command_thrusts(i);
                    end
            end
        end
          
        function body_torque = compute_body_torque_wheels(obj, wheel_torques)
            arguments
                obj Spacecraft
                wheel_torques (:,1) double
            end
            body_torque = obj.ReactionWheelMountingMatrix * (-wheel_torques);
        end

        function body_torque = compute_body_torque_thrusters(obj, thruster_thrusts)
            arguments
                obj Spacecraft
                thruster_thrusts (:,1) double
            end
            body_torque = obj.ThrusterMountingMatrix * (-thruster_thrusts);
        end

        function wheels_angular_momentum = compute_wheels_angular_momentum( ...
                obj, wheel_omegas)
            arguments
                obj Spacecraft
                wheel_omegas (1,:) double
            end
            if isempty(wheel_omegas)
                wheels_angular_momentum = zeros(3,1);
                return
            end

            wheels_angular_momentum = zeros(length(wheel_omegas),1);
            for i=1:length(obj.ReactionWheels)
                wheel_I = obj.ReactionWheels(i).I;
                wheels_angular_momentum(i) = wheel_I(1) * wheel_omegas(i);
            end
            wheels_angular_momentum = ...
                obj.ReactionWheelMountingMatrix*wheels_angular_momentum;
        end
    end
end

