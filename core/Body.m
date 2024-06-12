classdef Body
    %BODY
    
    properties
        Name string
        Mu (1,1) double {mustBeNonnegative}
        tEphemerisLookupTable (1,:) double
        rEphemerisLookupTable (:,3) double
        vEphemerisLookupTable (:,3) double
    end
    
    methods
        function obj = Body(name,t0,tspan)
            %BODY Construct an instance of this class
            arguments
                name 
                t0 datetime
                tspan (1,:) double
            end
            obj.Name = name;

            mu_earth = 398600.4418; % [km^3/sec^2]
            mu_moon = 4903; % [km^3/sec^2]
            mu_sun = 0; % Ignore Sun for gravity purposes
            mu_test_body = mu_moon; % Test body for circular simulations
            
            available_bodies = ["Earth","Moon","Sun","TestBody"];
            body_mus = [mu_earth,mu_moon,mu_sun,mu_test_body];
            body_mu_dict = dictionary(available_bodies, body_mus);

            obj.Mu = body_mu_dict(name);

            obj = obj.initializeEphemerides(t0,tspan);
        end

        function obj = initializeEphemerides(obj,t0,tspan)
            arguments
                obj Body
                t0 datetime
                tspan (1,:) double
            end

            if obj.Name == "TestBody"
                return
            end

            t_interps = tspan(1):1:tspan(end);
            ephemerisTime = zeros(length(t_interps),2);
            ephemerisTime(:,1) = juliandate(t0);
            ephemerisTime(:,2) = t_interps / 86400;

            [r,v] = planetEphemeris(ephemerisTime,'Earth',obj.Name);

            obj.tEphemerisLookupTable = t_interps;
            obj.rEphemerisLookupTable = r;
            obj.vEphemerisLookupTable = v;
        end

        function [r,v] = rvEarthBody(obj,t)
            arguments
                obj Body
                t double
            end
            if obj.Name == "TestBody"
                r = [0;0;0];
                v = [0;0;0];
                return
            end
            r = interp1(obj.tEphemerisLookupTable, ...
                obj.rEphemerisLookupTable,t)';
            v = interp1(obj.tEphemerisLookupTable, ...
                obj.vEphemerisLookupTable,t)';
        end

        function r = rEarthBody(obj, t)
            arguments
                obj Body
                t double
            end
            if obj.Name == "TestBody"
                r = [0;0;0];
                return
            end
            r = interp1(obj.tEphemerisLookupTable, ...
                obj.rEphemerisLookupTable,t)';
        end

        function v = vEarthBody(obj, t)
            arguments
                obj Body
                t double
            end
            if obj.Name == "TestBody"
                v = [0;0;0];
                return
            end
            v = interp1(obj.tEphemerisLookupTable, ...
                obj.rEphemerisLookupTable,t)';
        end
    end
end

