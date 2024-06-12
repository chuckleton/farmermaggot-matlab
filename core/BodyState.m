classdef BodyState
    %BODYSTATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Body Body
        RInertial (3,1) double
        VInertial (3,1) double
        QECI2RTN (4,1) double
        RotmECI2RTN (3,3) double
        SpacecraftI_RTN (3,3) double
    end
    
    methods
        function obj = BodyState(body,t,simulation,spacecraftState)
            arguments
                body Body
                t double
                simulation Simulation
                spacecraftState SpacecraftState
            end
            obj.Body = body;

            [x_eci, v_eci] = body.rvEarthBody(t);
            obj.RInertial = x_eci;
            obj.VInertial = v_eci;

            [qECI2RTN, rotmECI2RTN] = obj.spacecraftECI2RTN(spacecraftState);
            spacecraftI_RTN = obj.spacecraftIRTN(simulation,spacecraftState);

            obj.QECI2RTN = qECI2RTN;
            obj.RotmECI2RTN = rotmECI2RTN;
            obj.SpacecraftI_RTN = spacecraftI_RTN;
        end

        function [qECI2RTN, RotmECI2RTN] = spacecraftECI2RTN(obj, spacecraftState)
            arguments
                obj BodyState
                spacecraftState SpacecraftState
            end
            rBodyECI = obj.RInertial;
            vBodyECI = obj.VInertial;
            rSatECI = spacecraftState.RInertial;
            vSatECI = spacecraftState.VInertial;
            % Get all the axes for all the reference frames at each timestep
            satBodyRECI = rSatECI - rBodyECI;
            satBodyVECI = vSatECI - vBodyECI;
            satBodyVECIUnit = satBodyVECI / norm(satBodyVECI);
            RAxis = satBodyRECI / norm(satBodyRECI);
            NAxis = cross(RAxis, satBodyVECIUnit);
            TAxis = cross(NAxis,RAxis);
            TAxis = TAxis / norm(TAxis);

            RotmECI2RTN = [RAxis TAxis NAxis];

            qECI2RTN = rotm2quat(RotmECI2RTN);
        end

        function I_RTN = spacecraftIRTN(obj,simulation,spacecraftState)
            arguments
                obj BodyState
                simulation Simulation
                spacecraftState SpacecraftState
            end
            % Get the ECI2RTN rotation matrix
            [qECI2RTN,~] = obj.spacecraftECI2RTN(spacecraftState);
            I_body = simulation.Spacecraft.IBody;
            % RBody2ECI = quat2rotm(spacecraftState.QInertial')';

            % Rotate I_body body -> ECI
            % I_ECI = RBody2ECI * I_body * RBody2ECI';
            I_ECI = rotate_inertia_tensor(I_body,quatinv(spacecraftState.QInertial'));

            % Rotate I_ECI ECI -> RTN
            % I_RTN = RECI2RTN * I_ECI * RECI2RTN';
            I_RTN = rotate_inertia_tensor(I_ECI,qECI2RTN);
        end
    end
end

