classdef Surface
    %SURFACE
    
    properties
        Barycenter (3,1) double
        Area double
        Normal (3,1) double
        R_diff double = 0.002
        R_spec double = 0.002
        Cd double = 3.0
    end
    
    methods
        function obj = Surface(barycenter,area,normal)
            %SURFACE Construct an instance of this class
            arguments
                barycenter (3,1) double
                area double
                normal (3,1) double
            end
            obj.Barycenter = barycenter;
            obj.Area = area;
            obj.Normal = normal;
        end
    end
end

