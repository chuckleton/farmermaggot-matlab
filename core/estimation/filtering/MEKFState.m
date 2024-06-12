classdef MEKFState
    %MEKFSTATE
    
    properties
        QRef (4,1) double
        BetaRef (3,1) double
        DeltaX (6,1) double
        P (6,6) double
    end
    
    methods
        function obj = MEKFState(q,beta,deltaX,P)
            obj.QRef = q;
            obj.BetaRef = beta;
            obj.DeltaX = deltaX;
            obj.P = P;
        end
    end
end

