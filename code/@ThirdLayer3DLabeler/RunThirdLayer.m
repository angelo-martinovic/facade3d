

function RunThirdLayer(obj)
    tic;
    dl = DispatchingLogger.getInstance();
    
    datasetConfig = obj.config;
    inputName = obj.splitName; 
    
    dl.Log(VerbosityLevel.Debug,...
    sprintf('Running 3D third layer on top of %s point cloud labeling.\n',inputName));

    %--- function to check which points are inside box
    pts_in_bbox = @(pts_facade,tmp_corner) ...
        pts_facade(1,:)<tmp_corner(1) &...
        pts_facade(2,:)<tmp_corner(2) &...
        pts_facade(1,:)>tmp_corner(4) &...
        pts_facade(2,:)>tmp_corner(5);

    %--- read cprb from before results
    [~,~,rgb_data_cprb] = read_ply(get_adr('pcl_labeling',datasetConfig,inputName),'pcl');
    rgb_data_cprb = rgb_data_cprb(obj.pcl_test.p_index,:);
    cprb = Colors2Labels(rgb_data_cprb,datasetConfig.cm)';

    pts_split_label    = load([datasetConfig.dataLocation,datasetConfig.splitData]);
    obj.pcl_test.facade_id      = pts_split_label.splitLabels(obj.pcl_test.p_index)';
    
    %--- which facades to process
    facade_ids_go = unique(obj.pcl_test.facade_id);  %%% ids of facedes
    facade_ids_go(facade_ids_go==0)=[]; % Remove the background

    %--- what to export
    plot_boxes_while_optimizing             = 0;   %%% plot boxes before and after optimization :)
    export_result_to_full_obj.pcl_test_ply         = 1;   %%% for andelos's evaluation
    save_separate_rgb_projected3dfacades    = 1;   %%% each facade with boxes for projections to 3ds max:)
    visualize_full_obj.pcl_test                    = 0;   %%% show the entire re-labeled point cloud
    save_res_per_facade2file                = 1;   %%% if to save results
    
    dl.Log(VerbosityLevel.Info,...
    sprintf('- Optimizing for objects (window, balcony, door).\n'));

    facade_run_3Layer3D;


    %--- can be always on -> this defines what to calculate from 3rd layer boxes   
    store_bbox = 1;  %%% if I want to store all boxes (yes for export)
    estimate_door_box = 1; %%% always on!

    cmap = round(datasetConfig.cm(2:end,:)*255);

    %--- init some vars
    cprb_new_objs = cprb;
    if store_bbox, %%% if I may export to 3ds and want to have 
        pts = []; tri = []; tri_color = []; pts_facid=[]; tri_facid=[]; pts_ori=[]; pts_classid=[];tri_classid=[]; pts_oid=[]; tri_oid=[];
%         cmap = obj.pcl_test.get_class_colormap();cmap = cmap(2:end,:);     
    end
%     labeling_cprb_doors = [];
    if estimate_door_box, %%% door needs probability map, not just labeling...
        labeling_cprb_doors = obj.pcl_test.create_oidx_from_lidx('cprb',cprb,'K',7,'cl',4,'max_objs',100);
        labeling_cprb_doors = labeling_cprb_doors.oindex;
        prob_classes = load(get_adr('pcl_unaries',datasetConfig,inputName));
        %prob_classes = load(fullfile(path_cprb_probl,['3D_layer1-',num2str(2000),'-',num2str(200),'.mat']));
        prob_classes = prob_classes.unary(:,obj.pcl_test.origIndices);%prob_classes.prb(4:4+numel(obj.pcl_test.get_class_names),:);
        prob_classes = exp(-prob_classes) ./ repmat(sum(exp(-prob_classes)),size(prob_classes,1),1);
    end


    %==========================================================================
    %------- Now project boxes to pcl and update the probability map.
    %        Also estimate 3D boxes.
    %--------------------------------------------------------------------------
%     time_cumsum_3rd_end = 0;
    dl.Log(VerbosityLevel.Info,...
    sprintf('- Optimizing for stuff (sky, roof, shop, wall).\n'));
    %.Use precalculated bboxes of wind+balc, estimate rest, project it to pcl and show the result!'));
    for facade_id = facade_ids_go,
        if facade_id < 1,
            continue;
        end
        %--- read boxes
        idx_in_facade = find(obj.pcl_test.facade_id==facade_id);
        dl.Log(VerbosityLevel.Info,...
         sprintf('- - facade id=%i, #pts=%.1fK\n',facade_id,length(idx_in_facade)/1e3));

        path_data = get_adr('3DL_bboxes',datasetConfig,inputName,facade_id);
        if ~exist(path_data,'file');
            dl.Log(VerbosityLevel.Warning,sprintf('- - - No boxes estimated for this facade!\n'));
            continue;
        end
        t = dir(path_data);
        dl.Log(VerbosityLevel.Debug,...
            sprintf('- - Reading from %s, created at %s\n',path_data,t.date));
        load(path_data);  %%% read normals, bbox, bbox_ga and bbox_new
        %--- read grav. vector
        grav_vec = load(get_adr('splitPlane',datasetConfig,inputName,num2str(facade_id)));%load( get_adr('grav_vec',datasetConfig,facade_id) );
        grav_vec = grav_vec.g';

%         tic;

        %---- project optimization to facade's labeling -------------------
        [pts_facade,~] = project_in3d_pts_to_planes_normal( obj.pcl_test.pts(:,idx_in_facade) , nrm , [] , grav_vec);
        dl.Log(VerbosityLevel.Debug,...
            sprintf('- - # of objects: %i->%i->%i\n',length(bbox.id),length(bbox_ga.id),length(bbox_new.id)));

        % TODO: CLASSES ARE HARDCODED HERE!!!!
        %--- Now, lets do something with boxes... firstly reset to wal/sky/balc..
        tbox = facade_estimate_wall_bboxes(bbox_new,cprb,idx_in_facade,pts_facade,pts_in_bbox,labeling_cprb_doors,prob_classes);
        cprb_new_objs(idx_in_facade) = 2; %%% set everything to wall
        
        %---- now project classes that are not win. or balc.
        for co = [6 5 7 4];
            ids_local  = cprb(idx_in_facade)==co;  %%% idxs in facade
%             if sum(ids_local)==0; continue; end;
            tmp_corners = tbox.corners(:,tbox.class==co);
            if isempty(tmp_corners), continue; end;
            for obj=1:size(tmp_corners,2)
                idx_pts_in_corners = pts_in_bbox(pts_facade,tmp_corners(:,obj));
                cprb_new_objs(idx_in_facade(idx_pts_in_corners)) = co;
            end
            
        end

        %--- now project the value of each box (win+balc)
        for b=1:length(bbox_new.class),
            %--- pts in the bbox
            idx_pts_in_box_facade = pts_in_bbox(pts_facade,bbox_new.corners(:,b));
            idx_in_box = logical(obj.pcl_test.pts(1,:)*0);
            %--- assign value
            idx_in_box(idx_in_facade) = idx_pts_in_box_facade;
            cprb_new_objs(idx_in_box) = bbox_new.class(b);
        end
%         time_cumsum_3rd_end =  time_cumsum_3rd_end +toc;

        %--- calculation done, now just processing the data for
        %       visualization that is needed....


        %--- if you want to store all boxes together for furter visualization in 3ds max.
        if store_bbox, %     pts=[]; tri=[]; tri_color=[]; pts_facid=[]; tri_facid=[]; pts_ori=[];
            pts_move_mean = mean(obj.pcl_test.pts(:,idx_in_facade),2);
            move_pts_to_pos = @(pts_mean,pts,II) bsxfun(@plus,[pts_mean],[II(1,:)*pts ; II(2,:)*pts ; II(3,:)*pts]);
            get_wall        = @(pts) [min(pts(1:2,:),[],2) ; mean(pts(3,:))+0.01 ; max(pts(1:2,:),[],2) ; mean(pts(3,:))-0.01];
            [v1,v2]       = get_coorframe_given_normal(nrm);
            II            = inv([v1';v2';nrm']);  %%% calcualte back-projection from single facade to obj.pcl_test
            for cnt=1:length(tbox.class);
                [t_pts,t_qdr] = get_box_pos_from_its_cenAndSize(tbox.corners([3 6],cnt),tbox.corners([2 5],cnt),tbox.corners([1 4],cnt));
                cnt_p = size(pts,2);
                if tbox.class(cnt) == 5, %%% roof -> roate it to loosk cooler :)
                    t_pts(3,5:8) = t_pts(3,5:8)-0.2;
                end
                if isempty(pts_oid),  oid = 1; else oid=pts_oid(end)+1; end %%%  this is first time and it hearts... 
                pts = [pts move_pts_to_pos(pts_move_mean,t_pts,II)];
                tri = [tri [t_qdr(1:3,:) t_qdr([1 3 4],:)]+cnt_p];
                tri_color = [tri_color repmat(cmap(tbox.class(cnt),:)',1,size(t_qdr,2)*2)];
                pts_facid = [pts_facid t_pts(1,:)*0+facade_id];
                tri_facid = [tri_facid t_qdr(1,:)*0+facade_id t_qdr(1,:)*0+facade_id];
                pts_classid = [pts_classid t_pts(1,:)*0+tbox.class(cnt)];
                tri_classid = [tri_classid t_qdr(1,:)*0+tbox.class(cnt) t_qdr(1,:)*0+tbox.class(cnt)];
                pts_ori     = [pts_ori t_pts];
                pts_oid     = [pts_oid t_pts(1,:)*0+oid];
                tri_oid     = [tri_oid t_qdr(1,:)*0+oid  t_qdr(1,:)*0+oid];
            end
        end
        if plot_boxes_while_optimizing
            cam_set = @() eval(' axis equal off; campos([0 -1 3]); camup([-1 0 0]); camzoom(0.8); camproj(''orthographic''); set(gcf, ''Position'', [400 400 800 600]); set(gcf, ''Color'', [1 1 1]); zoom(1.2);');
            path2save = ['../output/export/3D_3L_facades_boxes/'];
            checkAdr_and_createDir(path2save);
%             cmap = obj.pcl_test.get_class_colormap();cmap = cmap(2:end,:);
            try close all; end;
            % hf=figure(); honza_scatter(pts_facade,pts_facade(1,:)*0+20,cmap(cprb(idx_in_facade),:)/255,'filled');  set(hf,'Position',[0,0,600,700]); set(gcf, 'Color', [1 1 1]); zoom(1.2); axis off equal; view(90,90);
            % im = screencapture(hf); delete(hf); imwrite(imcrop(im,[10 80 size(im,2)-20 size(im,1)-120]),[path2save,num2str(facade_id),'_',inputName,'_0prob.jpg']);
            try delete(hf); end;  hf=figure(); for cnt=1:length(bbox.class), col=[1 0 0];if bbox.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.35); hold on; end; cam_set();
            export_fig([path2save,num2str(facade_id),'_',inputName,'_1initial.jpg']);
            try delete(hf); end;   hf=figure(); for cnt=1:length(bbox_ga.class), col=[1 0 0];if bbox_ga.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox_ga.corners([3 6],cnt),bbox_ga.corners([2 5],cnt),bbox_ga.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.35); hold on; end;  cam_set();
            export_fig([path2save,num2str(facade_id),'_',inputName,'_2ga.jpg']);
            try delete(hf); end;   hf=figure(); for cnt=1:length(bbox_new.class), col=[1 0 0];if bbox_new.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox_new.corners([3 6],cnt),bbox_new.corners([2 5],cnt),bbox_new.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.35); hold on; end; cam_set();
            export_fig([path2save,num2str(facade_id),'_',inputName,'_3optim.jpg']);
            try delete(hf); end; close all;
            hf = figure(32); for cnt=1:length(tbox.class); col=cmap(tbox.class(cnt),:)/255; honza_plot_3d_cube(tbox.corners([3 6],cnt),tbox.corners([2 5],cnt),tbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',1); hold on; end; alpha(0.8); cam_set();
            export_fig([path2save,num2str(facade_id),'_',inputName,'_4fac.jpg']); delete(hf);
            dl.Log(VerbosityLevel.Debug,...
                sprintf(' - - Bbox saved to: %s - %s - facade ID=%i...\n',path2save,inputName,facade_id));
        end
    end


    dl.Log(VerbosityLevel.Info,sprintf('- Elapsed time for calculation: %.2f\n',toc));
    tic;

    %--- separate facades for 3ds max as BOXES with RGB projection :)
    if save_separate_rgb_projected3dfacades,
        dl.Log(VerbosityLevel.Info, sprintf('- Exporting results to 3ds Max.\n'));
        path_3ds_export_fac = '../output/export/3D_3L_facades_3ds/';
        for facade_id = facade_ids_go,

            im_path = get_adr('orthoLabels',datasetConfig,inputName,num2str(facade_id));
    %         im_path = ['/esat/sadr/amartino/monge428New/data/work/pcl/split/monge428New_fullRes_layer1+3D_3DCRF/monge428New_fullRes_split_',num2str(facade_id),'_ortho_labeling.png'];
            path_fac_img = sprintf('%s/%i.png',path_3ds_export_fac,facade_id);
            path_fac_obj = sprintf('%s/%i',path_3ds_export_fac,facade_id);

            im = imread(im_path);   %% read andelos image
            im = imrotate(im,90);
            checkAdr_and_createDir(path_fac_img);
            imwrite(im , path_fac_img);
            dl.Log(VerbosityLevel.Info, sprintf(' - - Copying image %s to texture %s\n',im_path,path_fac_img));

            %--- now 3D :) ----------------------------------------------------  
            %--- read angelo's datra to images and find corners
            dt_plane_andelo = load(get_adr('splitPlane',datasetConfig,inputName,num2str(facade_id)));
            im_corners = dt_plane_andelo.plane.b;

            boxFile = get_adr('3DL_bboxes',datasetConfig,inputName,facade_id);
            if ~exist(boxFile,'file');
                dl.Log(VerbosityLevel.Warning,sprintf('File %s does not exist!\n',boxFile));
                continue;
            end
            load(boxFile);  %%% read boxes for actual facade...
            [~,pts_mean]      = project_in3d_pts_to_planes_normal( obj.pcl_test.pts(:,obj.pcl_test.facade_id==facade_id) , nrm , [] , grav_vec);
            im_corners_proj   = project_in3d_pts_to_planes_normal(im_corners([1 3 5; 2 4 6]') ,nrm,pts_mean, grav_vec);

            %--- export sky, roof...
            move_in_x = -max(pts_ori(1,pts_facid==facade_id));  %%% fit to plane bottom
            facade_save_facades_boxes_for_3ds(pts_ori , tri , pts_facid==facade_id & pts_classid~=1 & pts_classid~=2 & pts_classid~=3 & pts_classid~=6,...
                tri_facid==facade_id & tri_classid~=1 & tri_classid~=2 & tri_classid~=3 & tri_classid~=6, im_corners_proj , [path_fac_obj,'.obj'],move_in_x);

            %--- save wall and windows as 3ds max defined bboxes so I could perform boolean opeartion later...   
            pts_box = pts_ori(:,pts_facid==facade_id & pts_classid==2);
            data2save = [mean(pts_box,2)' , max(pts_box,[],2)'-min(pts_box,[],2)']';
            id_wins_objs = pts_oid(pts_facid==facade_id & pts_classid==1);
            for id = unique(id_wins_objs),
                pts_box   = pts_ori(:,pts_oid==id); 
                data2save = [data2save , [mean(pts_box,2) ; max(pts_box,[],2)-min(pts_box,[],2)]];
            end
            data2save(1,:) = data2save(1,:)+move_in_x; %%% also swat to bottom plane...

            fid = fopen([path_fac_obj,'_wallWin.boxesbin'], 'w');
            fwrite(fid, size(data2save,1),'single');
            fwrite(fid, size(data2save,2),'single');
            fwrite(fid, data2save(:),'single'); %%% vertices + thir colorfacade_save_facades_boxes_for_3ds
            fclose(fid);
            
            dl.Log(VerbosityLevel.Info,...
                sprintf(' - - 3ds Max files expored to: %s_wallWin.boxesbin\n',path_fac_obj));

            %--- save balconies
            id2save = 3; facade_save_facades_boxes_for_3ds(pts_ori , tri , pts_facid==facade_id & pts_classid==id2save ,...
                tri_facid==facade_id & tri_classid==id2save , im_corners_proj , [path_fac_obj,'_balc.obj'],move_in_x);
        end
    end

    %--- project to full pcl (with background)  and save to andelo :)
    if export_result_to_full_obj.pcl_test_ply
        %--- project to full pcl
        cprb_full = zeros(1,size(obj.pcl_all.pts,2));
        cprb_full(obj.pcl_test.p_index) = cprb_new_objs;
        cmap = round(datasetConfig.cm*255);
        %--- visualize
        if visualize_full_obj.pcl_test
            colors_show = uint8((cmap(cprb_full+1,:)+full_pcl.rgb'*2)/3);
            full_pcl.plot_opengl_obj.pcl_test([full_pcl.lindex',cprb_full',cprb(i_pcl2full)'],'Q_rgb',colors_show);
        end

        path2save_full = get_adr('3D_L3_Pure3D_labeling',datasetConfig,inputName); 
        ExportMesh(path2save_full , obj.pcl_all.pts',[],cmap(cprb_full+1,:),[],[]);
        dl.Log(VerbosityLevel.Info,...
                sprintf('- - Result saved to %s as a point cloud for evaluation.\n',path2save_full));
    end
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Elapsed time for saving: %.2f\n',toc));

end
    