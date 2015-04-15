%
%
%
%
%
%    WA, part I
%
%    this function fits windows + balconies into a RF result
%
%         Honza Knopp, 2014
%
%
%
%
%
%
%





%cvx_setup   %%% you need cvx for optimization!!!
dl = DispatchingLogger.getInstance();




%--- init some vars
cprb_new_objs   = cprb*0;
cprb_new_neighs = cprb*0;
similar_principles = 0;
if ~exist('facade_ids_go','var') || isempty(facade_ids_go),
    facade_ids_go = unique(scene.facade_id);
end

time_cumsum = 0;
for facade_id = facade_ids_go,
    dl.Log(VerbosityLevel.Debug,...
    sprintf('- - Analyzing facade %d...\n',facade_id));
    grav_vec = load(get_adr('splitPlane',datasetConfig,inputName,num2str(facade_id)));%load(get_adr('grav_vec',datasetConfig,facade_id));
    
    grav_vec = grav_vec.g';
    if isempty(grav_vec)
        dl.Log(VerbosityLevel.Warning,sprintf('- - No gravity vector found.\n')); 
        continue; 
    end;
%     tic;
    idx_in_facade = find(scene.facade_id==facade_id);
    if length(idx_in_facade)<3e3, continue; end %%% not enough points in this segment -> it is small , so we need to compensate :)
    nrm = mean(scene.nxyz(:,idx_in_facade),2);
    nrm(2) = 0;
    [pts_facade,pts_mean] = project_in3d_pts_to_planes_normal( scene.pts(:,idx_in_facade) , nrm , [] , grav_vec);

    
    %======================================================================
    %   finally, we can start now...
    %----------------------------------------------------------------------
    
    
    %--- find super set of boxes --------
    bbox = facade_fit_bboxes_erode_into_prb_and_shrink(pts_facade,cprb,idx_in_facade,[1 3],pts_in_bbox);
    %   figure(); honza_scatter(pts_facade,pts_facade(1,:)*0+30,cprb(idx_in_facade),'filled'); hold on; for cnt=1:length(bbox.class), col=[1 0 0];if bbox.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.5); hold on; end;  cam_set(); title('initial hypos.');
    
    %--- now precalulate score for each box (# of pts insisde it...) for further calculations in ga/gradirent optmi 
    %    mat_bbox2pts_fac = sparse( length(bbox.class)  ,  size(pts_facade,2) );
    cprb_facade = cprb(idx_in_facade);
    bbox.num_pts_in = [];
    bbox.num_correct_pts_in = [];
    for b = 1:length(bbox.class)
        idx_pts_in_box_facade = logical(pts_in_bbox(pts_facade,bbox.corners(:,b)));
        bbox.num_correct_pts_in(b)      = sum(cprb_facade(idx_pts_in_box_facade)==bbox.class(b));
        bbox.num_pts_in(b)              = sum(idx_pts_in_box_facade);
    end
    %--- precompute size of window/balcony...
    for c=unique(bbox.class),
        median_size(1,c) = median(abs(bbox.corners(1,bbox.class==c)-bbox.corners(4,bbox.class==c)));
        median_size(2,c) = median(abs(bbox.corners(2,bbox.class==c)-bbox.corners(5,bbox.class==c)));
    end
    %--- find optimal subset
    bbox_ga = facade_lin_optim_get_superset(bbox, cprb(idx_in_facade) , median_size);

    %--- optimize the size and postion of each subset's box ------------
    bbox_new = facade_align_optim (bbox_ga ,  median_size);
    
    %--- remove small boxes...
    small_boxes = abs( (bbox_new.corners(1,:)-bbox_new.corners(4,:)).*(bbox_new.corners(2,:)-bbox_new.corners(5,:)) )  <   0.004;
    bbox_new.center  = bbox_new.center(:,~small_boxes);
    bbox_new.corners = bbox_new.corners(:,~small_boxes);
    bbox_new.class   = bbox_new.class(~small_boxes);
    bbox_new.id      = bbox_new.id(~small_boxes);

    
%     time_cumsum = time_cumsum+toc;
    if save_res_per_facade2file,
        path2save = get_adr('3DL_bboxes',datasetConfig,inputName,facade_id);
        dl.Log(VerbosityLevel.Debug,...
            sprintf('- - Saving estimated bounding boxes to %s\n',path2save));
        checkAdr_and_createDir(path2save);
        save(path2save,'bbox','bbox_ga','bbox_new','nrm');
        dl.Log(VerbosityLevel.Debug,...
            sprintf('- - - Done.\n'));
    end

    
    if 0,  %%% print boxes in 3d
        figure(); for cnt=1:length(bbox.class), col=[1 0 0];if bbox.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.4); hold on; end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]); zoom(1.2);
        figure(); for cnt=1:length(bbox_ga.class), col=[1 0 0];if bbox_ga.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox_ga.corners([3 6],cnt),bbox_ga.corners([2 5],cnt),bbox_ga.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.4); hold on; end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]); zoom(1.2);
        figure(); for cnt=1:length(bbox_new.class), col=[1 0 0];if bbox_new.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox_new.corners([3 6],cnt),bbox_new.corners([2 5],cnt),bbox_new.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.4); hold on; end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]); zoom(1.2);
    end
end


