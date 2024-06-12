function surfaces = sc_surfaces()
%% Central body surfaces
CB_center = [0 -0.157 0];
CB_l = 1.22;
CB_w = 0.76;
CB_h = 0.92;
CB_surfaces = get_rect_prism_surfaces(CB_center,CB_l,CB_w,CB_h);

%% Solar panel surfaces
SP1_center = [-1.081 0 0];
SP_l = 0.6;
SP_w = 1.2;
SP_h = 0.0;
SP_surfaces = get_rect_prism_surfaces(SP1_center,SP_l,SP_w,SP_h);
% We can ignore elements 3+ in the SP surfaces since it is 0-height
SP1_surfaces = SP_surfaces(1:2,:,:);

SP2_center = [1.081 0 0];
SP_surfaces = get_rect_prism_surfaces(SP2_center,SP_l,SP_w,SP_h);
% We can ignore elements 3+ in the SP surfaces since it is 0-height
SP2_surfaces = SP_surfaces(1:2,:,:);

%% Create a combined surfaces matrix
ss = [CB_surfaces;SP1_surfaces;SP2_surfaces];

%% Get the surface properties
for i=1:size(ss,1)
    [barycenter,surface_size,normal] = surface_props(squeeze(ss(i,:,:)));
    surfaces(i) = Surface(barycenter,surface_size,normal);
end
end

