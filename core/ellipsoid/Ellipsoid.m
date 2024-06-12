classdef Ellipsoid
    %ELLIPSOID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Center (3,1) double
        SemiAxes (3,1) double
    end
    
    methods
        function obj = Ellipsoid(center,semi_axes)
            %ELLIPSOID Construct an instance of this class
            obj.Center = center;
            obj.SemiAxes = semi_axes;
        end
        
        function plot(obj,facecolor,facealpha,label)
            [X,Y,Z] = ellipsoid(obj.Center(1), obj.Center(2), obj.Center(3), ...
            obj.SemiAxes(1), obj.SemiAxes(2), obj.SemiAxes(3));
            surf(X,Y,Z,'FaceColor',facecolor,'FaceAlpha',facealpha,'EdgeColor','none', ...
                'DisplayName',label);
        end
    end
end

