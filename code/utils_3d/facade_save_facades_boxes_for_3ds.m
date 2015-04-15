

function facade_save_facades_boxes_for_3ds(pts_ori,tri,ids_p,ids_t,im_corners_proj,path_fac_obj,move_in_x)


p = pts_ori(:,ids_p);
t = tri(:,ids_t);
if 1, %%% project considering gaps
    tidx_map = zeros(1,max(find(ids_p)));
    tidx_map(ids_p) = 1:sum(ids_p);
    t = tidx_map(t);
else %%%without gaps
    t = t-min(t(:))+1;
end

%--- add corners :)
p = [p im_corners_proj];
p(1,:) = p(1,:)+move_in_x;
num_p = size(p,2);
t = [t [num_p num_p num_p]'-1 [num_p num_p num_p]'];

%-- also shrink facade if overfit the image...
min2 = min(im_corners_proj(2,:));
max2 = max(im_corners_proj(2,:));
p(2,p(2,:)<min2) = min2;
p(2,p(2,:)>max2) = max2;

min1 = min(im_corners_proj(1,:)+move_in_x);
max1 = max(im_corners_proj(1,:)+move_in_x);
p(1,p(1,:)<min1) = min1;
p(1,p(1,:)>max1) = max1;


try
    path_tmp_file = '../tmp/t_fac.ply';
    ExportMesh(path_tmp_file',p',[],[],t',[]); 
    [~,~] = system(['unset LD_LIBRARY_PATH; meshlabserver -i ',path_tmp_file,' -o ',path_fac_obj]);
%     disp(['   ...full boboxes saved to ',path_fac_obj]);
catch
    error('We need to export to obj format, it is done by exporting to ply and meshlabserver converts to obj. Be sure you have meshlabserver in your path, or use soem obj exporter that can saves correctly for 3ds max.');
end



end