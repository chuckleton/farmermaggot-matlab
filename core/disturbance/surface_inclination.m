function inclination = surface_inclinations(surface_normal, vec)
% Gets inclinations of surface normal relative to vec
%INPUTS:
%   surface_normals: 3x1 matrix of unit surface normals
%   vec: 3x1 vector to calculate inclinations relative to
%OUTPUTS:
%   inclinations: 3x1 matrix of surface inclinations
inclination = dot(surface_normal,vec) / norm(vec);
end

