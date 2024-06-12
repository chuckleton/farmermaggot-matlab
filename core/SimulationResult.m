classdef SimulationResult
    properties
        Simulation Simulation
        StateVec (:,:) double
        SimulationStates SimulationState
        T (:,1) double
        RInertial (:,3) double
        VInertial (:,3) double
        QInertialBody (:,4) double  % ECI to Body
        QInertialPrincipal (:,1) quaternion % ECI to Principal
        QRTNInertial % Dictionary of RTN to Inertial quaternions
        QRTNBody % Dictionary of RTN to Body quaternions
        QRTNPrincipal % Dictionary of RTN to Principal quaternions
        OmegaInertial (:,3) double  % ECI
        OmegaBody (:,3) double
        OmegaPrincipal (:,3) double
        OmegaWheels (:,:) double
        LInertial (:,3) double % Angular momentum (ECI)
        LBody (:,3) double % Angular momentum (Body)
        LPrincipal (:,3) double % Angular momentum (Principal)
        TInertial (:,1) double  % Rotational KE (ECI)
        TBody (:,1) double  % Rotational KE (Body)
        TPrincipal (:,1) double  % Rotational KE (Principal)
        GravityGradientTorques (:,3) double % (ECI)
        MagneticTorques (:,3) double    % (ECI)
        SolarRadiationPressureTorques (:,3) double  % (ECI)
        AerodynamicPressureTorques (:,3) double % (ECI)
        DisturbanceTorques (:,3) double % (ECI)
        BodyPositions
        SatBodyPositions
        BodyVelocities
        TargetQInertial (:,4) double
        EstimatedQInertial (:,4) double
        TargetActualErrorQ (:,4) double
        EstimatedActualErrorQ (:,4) double
        EstimatedBeta (:,3) double
        AttitudeCovariance (:,3,3) double
        BetaCovariance (:,3,3) double
        MeasurementPreResiduals (:,:,3) double
        MeasurementPostSingleResiduals (:,:,3) double
        MeasurementPostOverallResiduals (:,:,3) double
        CommandTorques (:,3) double
        WheelTorques (:,:) double
        WheelsAppliedTorques (:,3) double
        ThrusterPWPFStates (:,:) double
        ThrusterCommandThrusts (:,:) double
        ThrusterAppliedThrusts (:,:) double
        ThrustersAppliedTorques (:,3) double
        AppliedTorques (:,3) double
    end
    
    methods
        function obj = SimulationResult(simulation,t,simulation_states)
            arguments
                simulation Simulation
                t double
                simulation_states SimulationState
            end
            % r_eci = states(:,1:3);
            % v_eci = states(:,4:6);
            % q = states(:,7:10);
            % omega = states(:,11:13);

            % for i=1:length(t)
            %     ti = t(i);
            %     simulation_states(i) = simulation_state_from_vec(simulation,ti,states(i,:));
            % end
            N = length(t);
            n_wheels = length(simulation.Spacecraft.ReactionWheels);
            obj.RInertial = zeros(N,3);
            obj.VInertial = zeros(N,3);
            obj.QInertialBody = zeros(N,4);
            obj.OmegaBody = zeros(N,3);
            obj.OmegaWheels = zeros(N,n_wheels);
            for i=1:length(t)
                obj.RInertial(i,:) = simulation_states(i).SpacecraftState.RInertial;
                obj.VInertial(i,:) = simulation_states(i).SpacecraftState.VInertial;
                obj.QInertialBody(i,:) = simulation_states(i).SpacecraftState.QInertial;
                obj.OmegaBody(i,:) = simulation_states(i).SpacecraftState.OmegaBody;
                obj.OmegaWheels(i,:) = simulation_states(i).SpacecraftState.WheelOmegas;
            end

            obj.Simulation = simulation;
            obj.T = t;
            % obj.RInertial = r_eci;
            % obj.VInertial = v_eci;
            % obj.QInertialBody = q;
            % obj.OmegaBody = omega;
            obj.SimulationStates = simulation_states;
            % obj.StateVec = states;

            obj = obj.computeDisturbanceTorques();
            obj = obj.computeBodyLocations();
            obj = obj.computeAttitudeAllFrames();
            obj = obj.computeAngularMomentumKineticEnergy();
            obj = obj.computeRTNQuaternions();
            obj = obj.computeControl();
        end

        function obj = computeDisturbanceTorques(obj)
            arguments
                obj SimulationResult
            end
            gg_torques = zeros(size(obj.RInertial));
            mag_torques = zeros(size(obj.RInertial));
            srp_torques = zeros(size(obj.RInertial));
            ap_torques = zeros(size(obj.RInertial));
            dist_torques = zeros(size(obj.RInertial));

            config = obj.Simulation.Config;

            for i=1:length(obj.SimulationStates)
                simulation_state = obj.SimulationStates(i);
                if config.GravityGradientTorqueEnabled
                    gg_torques(i,:) = gravity_gradient_torque(simulation_state);
                end
                if config.MagneticFieldTorqueEnabled
                    mag_torques(i,:) = magnetic_field_torque(simulation_state);
                end
                if config.SolarRadiationPressureTorqueEnabled
                    srp_torques(i,:) = solar_radiation_pressure_torque(simulation_state);
                end
                if config.AerodynamicTorqueEnabled
                    ap_torques(i,:) = aero_torque(simulation_state);
                end
                dist_torques(i,:) = disturbance_torques(simulation_state);
            end

            obj.GravityGradientTorques = gg_torques;
            obj.MagneticTorques = mag_torques;
            obj.SolarRadiationPressureTorques = srp_torques;
            obj.AerodynamicPressureTorques = ap_torques;
            obj.DisturbanceTorques = dist_torques;
        end

        function obj = computeBodyLocations(obj)
            arguments
                obj SimulationResult
            end
            bodies = obj.Simulation.Bodies;

            body_positions_dict = dictionary;
            sat_body_positions_dict = dictionary;
            body_velocities_dict = dictionary;

            for body=bodies
                body_positions = zeros(length(obj.T),3);
                sat_body_positions = zeros(length(obj.T),3);
                body_velocities = zeros(length(obj.T),3);
                for i=1:length(obj.T)
                    ti = obj.T(i);
                    [xi,vi] = body.rvEarthBody(ti);
                    sat_xi = obj.RInertial(i,:)';
                    body_positions(i,:) = xi;
                    sat_body_positions(i,:) = sat_xi - xi;
                    body_velocities(i,:) = vi;
                end
                body_positions_dict(body.Name) = {body_positions};
                sat_body_positions_dict(body.Name) = {sat_body_positions};
                body_velocities_dict(body.Name) = {body_velocities};
            end
            obj.BodyPositions = body_positions_dict;
            obj.SatBodyPositions = sat_body_positions_dict;
            obj.BodyVelocities = body_velocities_dict;
        end
    
        function obj = computeQPrincipal(obj)
            % Computes quaternion q from ECI to Principal axes
            spacecraft = obj.Simulation.Spacecraft;
            q_body_principal = quatinv(spacecraft.QPrincipalBody);
            q_inertial_body = quaternion(obj.QInertialBody);
            q_inertial_principal = quatmultiply(q_inertial_body,q_body_principal);

            obj.QInertialPrincipal = q_inertial_principal;
        end

        function obj = computeOmegaInertial(obj)
            omega_inertial = quatrotate(quatinv(obj.QInertialBody),obj.OmegaBody);
            obj.OmegaInertial = omega_inertial;
        end

        function obj = computeOmegaPrincipal(obj)
            spacecraft = obj.Simulation.Spacecraft;
            omega_principal = spacecraft.RPrincipalBody' * obj.OmegaBody';
            obj.OmegaPrincipal = omega_principal';
        end

        function obj = computeAttitudeAllFrames(obj)
            obj = obj.computeQPrincipal();
            obj = obj.computeOmegaInertial();
            obj = obj.computeOmegaPrincipal();
        end

        function obj = computeAngularMomentumKineticEnergy(obj)
            arguments
                obj SimulationResult
            end
            spacecraft = obj.Simulation.Spacecraft;
            I_inertial = rotate_inertia_tensor(spacecraft.IBody, ...
                quatinv(obj.QInertialBody));   % nx3x3

            % Compute L in principal axes
            L_principal = (spacecraft.IPrincipal * obj.OmegaPrincipal')';   % nx3

            % Compute L in body axes
            L_body = (spacecraft.IBody * obj.OmegaBody')';  % nx3


            % Compute L in inertial axes
            L_inertial = zeros(size(obj.OmegaInertial,1),3);
            for i = 1:size(obj.OmegaInertial,1)
                L_inertial(i,:) = squeeze(I_inertial(i,:,:))*obj.OmegaInertial(i,:)';    % nx3
            end
            
            % Update object
            obj.LInertial = L_inertial;
            obj.LBody = L_body;
            obj.LPrincipal = L_principal;

            % Compute Kinetic Energies
            T_inertial = 0.5*dot(obj.OmegaInertial, L_inertial,2);
            T_body = 0.5*dot(obj.OmegaBody, L_body,2);
            T_principal = 0.5*dot(obj.OmegaPrincipal, L_principal,2);

            obj.TInertial = T_inertial;
            obj.TBody = T_body;
            obj.TPrincipal = T_principal;
        end
    
        function obj = computeRTNQuaternions(obj)
            arguments
                obj SimulationResult
            end

            rtn_inertial_quaternions_dict = dictionary();
            rtn_body_quaternions_dict = dictionary();
            rtn_principal_quaternions_dict = dictionary();

            for body=obj.Simulation.Bodies
                rtn_inertial_quaternions = zeros(size(obj.T,1),4);
                rtn_body_quaternions = zeros(size(obj.T,1),4);
                rtn_principal_quaternions = zeros(size(obj.T,1),4);
                for i=1:size(obj.T,1)
                    simulation_state = obj.SimulationStates(i);
                    body_state = simulation_state.BodyStates(body.Name);
                    q_body = obj.QInertialBody(i,:);
                    q_principal = compact(obj.QInertialPrincipal(i));
                    q_ECI2RTN = body_state.QECI2RTN';
                    rtn_body_quaternions(i,:) = quatmultiply(quatinv(q_ECI2RTN),q_body);
                    rtn_principal_quaternions(i,:) = quatmultiply(quatinv(q_ECI2RTN),q_principal);
                    rtn_inertial_quaternions(i,:) = quatinv(q_ECI2RTN);
                    % rtn_principal_quaternions(i,:) = q_ECI2RTN;
                end
                rtn_inertial_quaternions_dict(body.Name) = {rtn_inertial_quaternions};
                rtn_body_quaternions_dict(body.Name) = {rtn_body_quaternions};
                rtn_principal_quaternions_dict(body.Name) = {rtn_principal_quaternions};
            end
            
            obj.QRTNInertial = rtn_inertial_quaternions_dict;
            obj.QRTNBody = rtn_body_quaternions_dict;
            obj.QRTNPrincipal = rtn_principal_quaternions_dict;
        end

        function obj = computeEstimation(obj)
            arguments
                obj SimulationResult
            end
            target_q = zeros(size(obj.T,1),4);
            estimated_q = zeros(size(target_q));
            attitude_cov = zeros(size(obj.T,1),3,3);
            beta_cov  = zeros(size(obj.T,1),3,3);
            for i=1:size(obj.T,1)
                target_q(i,:) = obj.SimulationStates(i).ControlState.TargetQInertial;
                estimated_q(i,:) = obj.SimulationStates(i).MekfIteration.XKPlus.QRef;
                attitude_cov(i,:,:) = obj.SimulationStates(i).MekfIteration.XKPlus.P(1:3,1:3);
                beta_cov(i,:,:) = obj.SimulationStates(i).MekfIteration.XKPlus.P(4:6,4:6);
            end

            target_q = match_quaternion_signs(obj.QInertialBody,target_q);
            target_actual_error_q = quaternion_error(target_q,obj.QInertialBody);

            estimated_q = match_quaternion_signs(obj.QInertialBody,estimated_q);
            estimated_actual_error_q = quaternion_error(estimated_q,obj.QInertialBody);
            
            obj.TargetQInertial = target_q;
            obj.EstimatedQInertial = estimated_q;
            obj.TargetActualErrorQ = target_actual_error_q;
            obj.EstimatedActualErrorQ = estimated_actual_error_q;
            obj.AttitudeCovariance = attitude_cov;
            obj.BetaCovariance = beta_cov;

            % Get measurement residuals
            n_meas = length(obj.Simulation.Spacecraft.UnitVectorSensors);
            meas_pre_residuals = zeros(size(obj.T,1),n_meas,3);
            meas_post_single_residuals = zeros(size(meas_pre_residuals));
            meas_post_overall_residuals = zeros(size(meas_pre_residuals));

            estimated_beta = zeros(size(obj.T,1),3);

            for i=2:size(obj.T,1)
                for j=1:n_meas
                    meas_pre_residuals(i,j,:) = ...
                        obj.SimulationStates(i).MekfIteration.MeasurementIterations(j).MeasurementResidualPre;
                    meas_post_single_residuals(i,j,:) = ...
                        obj.SimulationStates(i).MekfIteration.MeasurementIterations(j).MeasurementResidualPostSingle;
                    meas_post_overall_residuals(i,j,:) = ...
                        obj.SimulationStates(i).MekfIteration.MeasurementIterations(j).MeasurementResidualPostOverall;
                end

                estimated_beta(i,:) = obj.SimulationStates(i).MekfIteration.XKPlus.BetaRef;
            end

            obj.MeasurementPreResiduals = meas_pre_residuals;
            obj.MeasurementPostSingleResiduals = meas_post_single_residuals;
            obj.MeasurementPostOverallResiduals = meas_post_overall_residuals;

            obj.EstimatedBeta = estimated_beta;
        end

        function obj = computeControl(obj)
            arguments
                obj SimulationResult
            end
            command_torques = zeros(size(obj.RInertial));
            n_wheels = length(obj.Simulation.Spacecraft.ReactionWheels);
            n_thrusters = length(obj.Simulation.Spacecraft.Thrusters);
            wheel_torques = zeros(size(command_torques,1),n_wheels);
            wheels_applied_torques = zeros(size(obj.RInertial));
            thruster_pwpf_states = zeros(size(command_torques,1),n_thrusters);
            thruster_command_thrusts = zeros(size(command_torques,1),n_thrusters);
            thruster_applied_thrusts = zeros(size(thruster_command_thrusts));
            thrusters_applied_torques = zeros(size(obj.RInertial));
            applied_torques = zeros(size(obj.RInertial));
            for i=1:length(obj.SimulationStates)
                control_state = obj.SimulationStates(i).ControlState;
                command_torques(i,:) = control_state.CommandTorque;
                wheel_torques(i,:) = control_state.WheelTorques;
                wheels_applied_torques(i,:) = control_state.WheelsAppliedTorque;
                thruster_pwpf_states(i,:) = control_state.ThrusterPWPFState;
                thruster_command_thrusts(i,:) = control_state.ThrusterCommandThrusts;
                thruster_applied_thrusts(i,:) = control_state.ThrusterAppliedThrusts;
                thrusters_applied_torques(i,:) = control_state.ThrustersAppliedTorque;
                applied_torques(i,:) = control_state.AppliedTorque;
            end
            obj.CommandTorques = command_torques;
            obj.WheelTorques = wheel_torques;
            obj.WheelsAppliedTorques = wheels_applied_torques;
            obj.ThrusterPWPFStates = thruster_pwpf_states;
            obj.ThrusterCommandThrusts = thruster_command_thrusts;
            obj.ThrusterAppliedThrusts = thruster_applied_thrusts;
            obj.ThrustersAppliedTorques = thrusters_applied_torques;
            obj.AppliedTorques = applied_torques;
        end
    end
end

