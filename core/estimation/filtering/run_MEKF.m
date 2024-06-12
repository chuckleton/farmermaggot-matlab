function estimated_result = run_MEKF(simulation_result)
arguments (Input)
    simulation_result SimulationResult
end
arguments (Output)
    estimated_result SimulationResult 
end
    estimated_q = zeros(size(simulation_result.QInertialBody));
    estimated_q(1,:) = simulation_result.QInertialBody(1,:);
    estimated_beta = zeros(size(estimated_q,1),3);
    delta_t = simulation_result.T(2) - simulation_result.T(1);

    P = zeros(size(estimated_q,1),6,6);
    P(1,:,:) = [0.1^2*eye(3) zeros(3,3);zeros(3,3) zeros(3,3)];

    G=[-eye(3) zeros(3);zeros(3) eye(3)];

    sigma_v = 0.01;
    sigma_u = 0;

    Q_init = zeros(6,6);
    Q_init(1:3,1:3) = (delta_t*sigma_v^2+delta_t^3*sigma_u^2/3);
    Q_init(1:3,4:6) = (0.5*delta_t^2*sigma_u^2);
    Q_init(4:6,1:3) = Q_init(1:3,4:6)';
    Q_init(4:6,4:6) = delta_t*sigma_u^2;
    Q=G*Q_init*G';

    for i=1:length(simulation_result.T)-1
        state = simulation_result.SimulationStates(i);
        omega = state.SpacecraftState.OmegaBody - estimated_beta(i,:)';

        % Add some noise to omega
        % omega_noise = mvnrnd(omega',0.01^2*eye(3))';
        % omega_bias = [0.001;0;0];
        % omega = omega + omega_noise + omega_bias;

        %% Propagate
        [q_minus, beta_minus, P_minus] = propagate_MEKF( ...
            estimated_q(i,:),estimated_beta(i,:),omega,P(i,:,:),Q,delta_t);

        %% Measurement Update
        % bogus for now
        q_plus = q_minus;
        beta_plus = beta_minus;
        P_plus = P_minus;

        estimated_q(i+1,:) = q_plus;
        estimated_beta(i+1,:) = beta_plus;
        P(i+1,:,:) = P_plus;
    end

    simulation_result.EstimatedQInertial = estimated_q;
    simulation_result.EstimatedBeta = estimated_beta;

    simulation_result.AttitudeCovariance = P(:,1:3,1:3);
    simulation_result.BetaCovariance = P(:,4:6,4:6);

    estimated_result = simulation_result.computeControl();
end

