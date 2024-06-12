function [n,V,p,max_dist] = best_fit_plane(X)
    %Computes best-fit plane for a set of sample points.
    %INPUTS:
    %   X: a N by 3 matrix in which each row is a sample point
    %OUTPUTS:
    %   n : a unit (column) vector normal to the plane
    %   V : a 3 by 2 matrix, with the columns space being a basis for the
    % plane
    %   p: point on the plane

    % Mean of samples is in the best-fit (2-norm) plane
    p = mean(X,1);
    
    % Get the samples with the mean subtracted
    R = bsxfun(@minus,X,p);
    % Get the eigenvectors of R^TR
    [V,~] = eig(R'*R);
    n = V(:,1);
    V = V(:,2:end);
    % Get the maximum deviation from the plane
    max_dist = 0;
    for i=1:size(X,1)
        point = X(i,:);
        dist = dot(point-p,n);
        if dist > max_dist
            max_dist = dist;
        end
    end
end