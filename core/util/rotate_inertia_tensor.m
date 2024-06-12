function I_prime = rotate_inertia_tensor(I,q)
% Transforms an inertia tensor I by a quaternion q
%INPUTS:
%   I: 3x3 Inertia tensor
%   q: nx1 quaternion(s), or nx4 matrix
%OUTPUTS:
%   I_prime nx3x3 transformed inertia tensor(s) (3x3 if n=1)
    n = size(q,1);

    %% Convert q to Rotation matrices
    R = quat2rotm(q); % 3x3xn

    %% Transform for n=1
    if n==1
        I_prime = R'*I*R;
        return
    end

    R_transpose = permute(R,[2 1 3]);   % 3x3xn
    
    %% Transform inertia tensor
    I_prime = zeros(size(R,3),3,3);
    for i=1:size(R,3)
        % Note we do R'IR instead of RIR' as normal
        % since matlab does reversed rotation matrices
        I_prime(i,:,:) = R_transpose(:,:,i)*I*R(:,:,i);
    end
end

