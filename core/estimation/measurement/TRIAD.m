function q_est = TRIAD(measurements,use_ficticious)
%TRIAD TRIaxial Attitude Determination algorithm
%   See eq 5.6 in Markley & Crassidis
%INPUTS:
%   measurements: 1x2 vector of UnitVectorMeasurements
%OUTPUTS:
%   q_est: Estimated attitude quaternion
arguments
    measurements (1,2) UnitVectorMeasurement
    use_ficticious logical = 1
end

m1 = measurements(1);
m2 = measurements(2);

r1 = m1.ReferenceAxis;
r2 = m2.ReferenceAxis;
b1 = m1.BodyAxis;
b2 = m2.BodyAxis;

% If we are using ficticious measurements, update r1 and r2
if use_ficticious
    r1_fict = (r1+r2) / 2;
    r2_fict = (r1-r2) / 2;
    b1_fict = (b1+b2) / 2;
    b2_fict = (b1-b2) / 2;
    r1 = r1_fict;
    r2 = r2_fict;
    b1 = b1_fict;
    b2 = b2_fict;
end

v1 = r1;
r_cross = cross(r1,r2) / norm(cross(r1,r2));
v3 = cross(r1,r_cross);

w1 = b1;
b_cross = cross(b1,b2) / norm(cross(b1,b2));
w3 = cross(b1,b_cross);

M = [v1 r_cross v3];
V = [w1 b_cross w3];    % Note: Orthonormal -> inv=transpose

A_est = M*V';
q_est = rotm2quat(A_est);
end

