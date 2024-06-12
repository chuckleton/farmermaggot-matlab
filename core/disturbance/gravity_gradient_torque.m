function tau_gg_ECI = gravity_gradient_torque( ...
    simulationState ...
)
% Gets gravity gradient torque on body
%INPUTS:
%   simulationState: SimulationState object
%OUTPUTS:
%   tau_gg: 3x1 vector of gravity gradient torque
arguments
    simulationState SimulationState
end
    simulation = simulationState.Simulation;
    spacecraft = simulation.Spacecraft;

    tau_ggs = zeros(simulation.NBodies,3);
    for i=1:simulation.NBodies
        body = simulation.Bodies(i);
        bodyState = simulationState.BodyStates(body.Name);

        mu = body.Mu;
        r_body = bodyState.RInertial;
        % Get the spacecraft inertia tensor in inertial coordinates
        I_inertial = rotate_inertia_tensor( ...
            spacecraft.IBody, ...
            quatinv(simulationState.SpacecraftState.QInertial') ...
        );
        % I_RTN = bodyState.SpacecraftI_RTN;

        r = simulationState.SpacecraftState.RInertial - r_body; % 3x1 nadir vector
        r_mag = norm(r); % 1x1
        n = r / r_mag; % nadir unit vector

        tau_gg_body_RTN = cross((3*mu / r_mag^3)*n,I_inertial*n);

        % Rotate to ECI frame
        tau_gg_body_ECI = bodyState.RotmECI2RTN' * tau_gg_body_RTN;
        tau_ggs(i,:) = tau_gg_body_ECI;
    end

    % Overall gravity gradient torque is sum of torque from each body
    tau_gg_ECI = sum(tau_ggs,1)';
end

