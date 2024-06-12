function q2_matched = match_quaternion_signs(q1,q2)
%MATCH_QUATERNION_SIGNS Match the signs of q2 to q1
%   Note: quaternions are assumed scalar-first per Matlab convention
%   To input/output from 1xn arrays of quaternion objects use:
%       q2_matched = quaternion(match_quaternion_signs(compact(q1),compact(q2)))
%INPUTS:
%   q1: Quaternion to match signs with
%   q2: Quaternion to modify to have matched signs
%OUTPUTS
%   q2_matched: q2 with signs flipped as needed to match scalar signs of q1
arguments (Input)
    q1 (:,4) double {mustBeReal}
    q2 (:,4) double {mustBeReal}
end
arguments (Output)
    q2_matched (:,4) double {mustBeReal}
end
    sign_q1_scalar = sign(q1(:,1));
    sign_q1_scalar(sign_q1_scalar==0) = 1;
    sign_q2_scalar = sign(q2(:,1));
    sign_q2_scalar(sign_q2_scalar==0) = 1;
    q2_matched = q2.*(sign_q1_scalar.*sign_q2_scalar);
end

