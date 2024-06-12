function [Ip,Ap,Rp0] = inertia_tensor_to_principal(I0, A0)
% Gets principal inertia tensor Ip, principal axes, 
% and rotation matrix from the given I0 to these axes
%INPUTS:
%   I0: The original inertia tensor in some axis frame
%   A0: The original axes
%OUTPUTS:
%   Ip: principal inertia tensor (diagonal)
%   Ap: principal axes (in the frame of the original axes A0)
%   Rp0: Rotation matrix from the principal axes to initial axes
%% Get the eigenvalue and vectors of I0
% Eigenvalues Ip are principal moments of inertia (diagonal matrix)
% Eigenvectors R0p are the rotation matrix from A0 to Ap
[Rp0, Ip] = eig(I0);

% Ap is A0 rotated by Rp0'
Ap = Rp0'*A0;
end

