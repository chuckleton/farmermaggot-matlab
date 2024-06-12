classdef Thruster
    %THRUSTER
    
    properties
        R (3,1) double
        E (3,1) double
        T (1,1) double
        SigmaCommand double
    end
    
    methods
        function obj = Thruster( ...
                r,e,t,sigma_command)
            %REACTIONWHEEL Construct an instance of this class
            arguments
                r (3,1) double
                e (3,1) double
                t double = 4
                sigma_command double = 0.04
            end
            obj.R = r;
            obj.E = e;
            obj.T = t;
            obj.SigmaCommand = sigma_command;
        end
    end
end

