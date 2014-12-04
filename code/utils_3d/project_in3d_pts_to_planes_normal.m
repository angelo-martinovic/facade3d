%==========================================================================
%--------------------------------------------------------------------------
%
%
%
% honza knopp
%
%
function [pts_out,pts_mean] = project_in3d_pts_to_planes_normal(pts,plane_normal,pts_mean,grav_vec)


if exist('grav_vec','var') && ~isempty(grav_vec),
    v1 = grav_vec;
    v2 = cross(plane_normal,v1);
else
    [v1,v2] = get_coorframe_given_normal(plane_normal);
end

pts=[v1'*pts ; v2'*pts ; plane_normal'*pts];

if ~exist('pts_mean','var') || isempty(pts_mean)
    pts_mean = mean(pts,2);
end

pts_out=bsxfun(@plus,pts,-pts_mean);
         


end


