classdef Simulation
    %SIMULATION
    
    properties
        Spacecraft Spacecraft
        Bodies (1,:) Body
        NBodies (1,1) int64
        Mekf MEKF
        Pwpf PWPF
        T0 datetime
        Tspan (1,:) double
        Config SimulationConfig
        Result SimulationResult
    end
    
    methods
        function obj = Simulation(spacecraft,bodyNames,mekf,pwpf,t0,tspan,config)
            %SIMULATION Construct an instance of this class
            arguments
                spacecraft Spacecraft
                bodyNames (1,:)
                mekf MEKF
                pwpf PWPF
                t0 datetime
                tspan (1,:) double
                config SimulationConfig
            end
            % Create the bodies
            for i=1:length(bodyNames)
                bodyName = bodyNames(i);
                bodies(i)=Body(bodyName, t0, tspan);
            end

            obj.Spacecraft = spacecraft;
            obj.Bodies = bodies;
            obj.NBodies = length(bodies);
            obj.Mekf = mekf;
            obj.Pwpf = pwpf;
            obj.T0 = t0;
            obj.Tspan = tspan;
            obj.Config = config;
        end

        function obj = simulate(obj)
            arguments
                obj Simulation
            end
            options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
            initial_state = obj.Spacecraft.InitialState.full_state_vec();

            state = simulate_rk4(obj);
            
            % [t,state] = ode113( ...
            %     @(t,y) state_derivative(t,y,obj),...
            %     obj.Tspan, ...
            %     initial_state, ...
            %     options...
            % );
            % 
            result = SimulationResult(obj,obj.Tspan,state);
            obj.Result = result;
            % obj.Mekf = obj.Mekf.run(result);
            obj.Result.Simulation = obj;
            obj.Result = obj.Result.computeEstimation();
        end

        function obj = estimate(obj)
            arguments (Input)
                obj Simulation
            end
            arguments (Output)
                obj Simulation
            end
            result = obj.Result;
            obj.Result = run_MEKF(result);
        end

        function rs = rEarthBodies(obj, t, t0)
            arguments
                obj Simulation
                t double
                t0 datetime
            end
            rs = zeros(obj.NBodies,3);
            for i=1:simulation.NBodies
                body = obj.Bodies(i);
                rs(i,:) = body.rEarthBody(t,t0);
            end
        end

        function mus = muBodies(obj)
            arguments
                obj Simulation
            end
            mus = zeros(obj.NBodies,1);
            for i=1:simulation.NBodies
                body = obj.Bodies(i);
                mus(i,1) = body.Mu;
            end
        end


        function [qECI2RTNs, RotmECI2RTNs] = spacecraftECI2RTNBodies( ...
                obj, t, t0, spacecraftState)
            arguments
                obj Simulation
                t double
                t0 datetime
                spacecraftState SpacecraftState
            end
            qECI2RTNs = zeros(obj.NBodies,4);
            RotmECI2RTNs = zeros(obj.NBodies,3,3);
            for i=1:simulation.NBodies
                body = obj.Bodies(i);
                [qECI2RTN,RotmECI2RTN] = body.spacecraftECI2RTN( ...
                    t, t0, spacecraftState);
                qECI2RTNs(i,:) = qECI2RTN;
                RotmECI2RTNs(i,:,:) = RotmECI2RTN;
            end
        end

        function I_RTNs = I_RTNBodies(obj,t,t0,spacecraftState)
            arguments
                obj Simulation
                t double
                t0 datetime
                spacecraftState SpacecraftState
            end
            I_RTNs = zeros(obj.NBodies,3,3);
            for i=1:simulation.NBodies
                body = obj.Bodies(i);
                I_RTNs(i,:,:) = body.spacecraftIRTN(t,t0,spacecraftState);
            end
        end
    end
end

