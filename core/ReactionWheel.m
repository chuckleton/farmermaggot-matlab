classdef ReactionWheel
    %REACTIONWHEEL
    
    properties
        Axis (3,1) double
        I (2,1) double
        MaxTorque double
        SigmaCommand double
    end
    
    methods
        function obj = ReactionWheel( ...
                axis,I,max_torque,sigma_command)
            %REACTIONWHEEL Construct an instance of this class
            arguments
                axis (3,1) double
                I double
                max_torque double = 0.25
                sigma_command double = 0.002
            end
            obj.Axis = axis;
            obj.I = I;
            obj.MaxTorque = max_torque;
            obj.SigmaCommand = sigma_command;
        end
    end
end

