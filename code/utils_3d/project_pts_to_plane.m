%==========================================================================
%--------------------------------------------------------------------------
%
%
%
% honza knopp
%
%
function pts_out = project_pts_to_plane(pts,plane_X,plane_normal)



%%

dist2plane    = plane_normal' * bsxfun(@plus , pts , -plane_X);
pts_out       = pts - bsxfun(@times , dist2plane , plane_normal);


end


