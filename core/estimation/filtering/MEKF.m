classdef MEKF
    %MEKF
    
    properties
        InitialState MEKFState
        Iterations (1,:) MEKFIteration
        SigmaU (1,1) double
        SigmaV (1,1) double
        G (6,6) double;
        Q (6,6) double;
        DeltaT (1,1) double;
    end
    
    methods
        function obj = MEKF(x0,sigma_u,sigma_v,delta_t)
            %MEKF Construct an instance of this class
            obj.InitialState = x0;
            obj.Iterations = MEKFIteration.empty();
            obj.SigmaU = sigma_u;
            obj.SigmaV = sigma_v;
            obj.DeltaT = delta_t;

            obj.G = [-eye(3) zeros(3);zeros(3) eye(3)];
            obj = obj.computeQ();
        end
        
        function obj = computeQ(obj)
            Q_init = zeros(6,6);
            Q_init(1:3,1:3) = (obj.DeltaT*obj.SigmaV^2+obj.DeltaT^3*obj.SigmaU^2/3);
            Q_init(1:3,4:6) = (0.5*obj.DeltaT^2*obj.SigmaU^2);
            Q_init(4:6,1:3) = Q_init(1:3,4:6)';
            Q_init(4:6,4:6) = obj.DeltaT*obj.SigmaU^2;
            obj.Q = obj.G*Q_init*obj.G';
        end

        function obj = run(obj, simulation_result)
            last_MEKF_state = obj.InitialState;
            obj.Iterations(1) = MEKFIteration( ...
                last_MEKF_state, ...
                simulation_result.SimulationStates(1).SpacecraftState.OmegaMeasurement.Omega, ...
                simulation_result.SimulationStates(2).SpacecraftState.UnitVectorMeasurements, ...
                obj.Q,obj.DeltaT);
            obj.Iterations(1).XKPlus = obj.InitialState;
            for i=1:length(simulation_result.T)-1
                omega = ...
                    simulation_result.SimulationStates(i).SpacecraftState.OmegaMeasurement.Omega;
                measurements = ...
                    simulation_result.SimulationStates(i+1).SpacecraftState.UnitVectorMeasurements;
                iteration = MEKFIteration( ...
                    last_MEKF_state, ...
                    omega,measurements, ...
                    obj.Q,obj.DeltaT);
                iteration = iteration.propagate();
                iteration = iteration.measurementUpdate();
                iteration = iteration.reset();
                last_MEKF_state = iteration.XKPlus;
                obj.Iterations(i+1) = iteration;
            end
        end

        function iteration = iterate( ...
                obj, previous_iteration, omega, measurements)
            arguments
                obj MEKF
                previous_iteration MEKFIteration
                omega (3,1) double
                measurements UnitVectorMeasurement
            end
            last_MEKF_state = previous_iteration.XKPlus;
            iteration = MEKFIteration( ...
                    last_MEKF_state, ...
                    omega,measurements, ...
                    obj.Q,obj.DeltaT);
            iteration = iteration.propagate();
            iteration = iteration.measurementUpdate();
            iteration = iteration.reset();
        end
    end
end

