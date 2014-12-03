

function bbox_new = facade_align_optim (bbox ,  median_size)

bbox_new = bbox;
for c2optim = [1 3], %%% optim each class independently
    for dim2optim = [1 4],
        bbox_new.corners(dim2optim,bbox.class==c2optim)  =  honza_optimize_bbox_postions( bbox.corners(dim2optim,bbox.class==c2optim) , median_size(1,c2optim)/2 );
    end
    for dim2optim = [2 5],
        bbox_new.corners(dim2optim,bbox.class==c2optim)  =  honza_optimize_bbox_postions( bbox.corners(dim2optim,bbox.class==c2optim) , median_size(2,c2optim)/2 );
    end
end
%--- z direction -> just to looks nice :)
bbox_new.corners([3 6],:)  =  honza_optimize_bbox_postions( bbox.corners([3 6],:) , median_size(1,1) , bbox);




end





