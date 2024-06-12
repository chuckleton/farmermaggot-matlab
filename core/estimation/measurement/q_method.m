function q_est = q_method(measurements)
%TRIAD TRIaxial Attitude Determination algorithm
%   See section 5.3.1 in Markley & Crassidis
%INPUTS:
%   measurements: 1xN vector of UnitVectorMeasurements
%OUTPUTS:
%   q_est: Estimated attitude quaternion
arguments
    measurements (1,:) UnitVectorMeasurement
end

N = length(measurements);

B = zeros(3,3);
z = zeros(3,1);
for i=1:N
    meas = measurements(i);
    B = B + meas.Sensor.Weight * meas.BodyAxis * meas.ReferenceAxis';
    z = z + meas.Sensor.Weight * cross(meas.BodyAxis,meas.ReferenceAxis);
end

K = [B+B'-trace(B)*eye(3) z;z' trace(B)];
[V,~] = eigs(K,1,'largestreal'); % Eigenvector corresponding to largest eigenvalue

% Represent as scalar first per Matlab convention
q_est = zeros(4,1);
q_est(1) = V(4);
q_est(2:4) = V(1:3);
end