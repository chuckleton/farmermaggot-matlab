classdef SimulationConfig
    %SIMULATIONCONFIG
    
    properties
        GravityGradientTorqueEnabled logical = true;
        AerodynamicTorqueEnabled logical = true;
        SolarRadiationPressureTorqueEnabled logical = true;
        MagneticFieldTorqueEnabled logical = true;
        ControllerType = "sliding";
    end
    
    methods
        function obj = SimulationConfig()
        end
    end
end

