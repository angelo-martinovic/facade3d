%%   initialization

setup;

datasetConfig = InitializeDataset('monge428');

% angelo
% Input: images and labels
% Output: classified images of the test set and the projected classification on the
% point cloud
FirstLayer2D(datasetConfig);

% honza
% Input: point cloud
% Output: classified point cloud
FirstLayer3D(datasetConfig);

% angelo
input = '2D+3D'; % '2D', '3D'
% Input: classified point clouds
% Output: combined classified point cloud 
CombineLabelings(datasetConfig,input);

% angelo
% Input: classified point cloud
% Output: improved classification of point cloud
ThirdLayer2D(datasetConfig,input);

% honza
% Input: classified point cloud
% Output: improved classification of point cloud
ThirdLayer3D(datasetConfig,input);





%% ========================================================================
%
%      3D: RF=Layer 1
%
%  ========================================================================

desc_reco      = 'desc'; %%% which descriptor, desc=concatenation of all descs.  
unary_clssifer = 'rf';   %%% classifier, RF=random forest
sample     = 10;2e3;   %%% # of samples per class
nrf_tree   = 10;200;   %%% # of trees in RF


%--- store only descriptors with labels, then learn classifier and then classfiy data  
[X,Y,Xtest] = facade_unary_class('get_desc','desc_reco',desc_reco,'unary_clssifer',unary_clssifer,'sample',sample,'norm_data',1);
model       = facade_unary_class('learn','unary_clssifer',unary_clssifer,'X',X,'Y',Y,'rf_trees',nrf_tree);
[prb,cprb]  = facade_unary_class('test','unary_clssifer',unary_clssifer,'model',model,'Xtest',Xtest);



%--- plot results in matlab, only the sub-sampled set as matlab is slow...
if 0,
    scene.plot_scene('sample',10,'Q',cprb');
    scene.plot_scene('sample',10,'Q',scene.lindex');
end















%% ========================================================================
%
%      3D: Weak arch rules... = Layer 3
%
%  ========================================================================

input_type_into_3rd = 'layer1+3D_3DCRF';'layer1+3D_3DCRF';%'layer1+3D_3DCRF';'layer1_3DCRF';'3D_3DCRF';'layer1+3D_3DCRF';'3D_3DCRF';'layer1+3D_3DCRF';'layer1_3DCRF';'layer1+3D_3DCRF';'3D_3DCRF';  % 3D_3DCRF  layer1+3D_3DCRF  layer1_3DCRF


path_cprbs_results = '/esat/sadr/amartino/monge428New/data/work/pcl/models/';
path_grav_vec      = '/esat/sadr/amartino/monge428New/data/work/pcl/split/monge428New_fullRes_layer1+3D_3DCRF/';
path_cprb_probl    = '/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428_27/res2angelo/';
path_images_source = path_grav_vec;
path_im_data_file  = '/esat/sadr/amartino/monge428New/data/work/pcl/split/monge428New_fullRes_layer1_3DCRF/';


%--- fce to check which points are inside box
pts_in_bbox = @(pts_facade,tmp_corner) pts_facade(1,:)<tmp_corner(1) & pts_facade(2,:)<tmp_corner(2) & pts_facade(1,:)>tmp_corner(4) & pts_facade(2,:)>tmp_corner(5);



%--- read cprb from before
path_input_type_into_3rd = fullfile(path_cprbs_results,['monge428New_',input_type_into_3rd,'.ply']);
[~,~,vertexColData] = read_ply(path_input_type_into_3rd,'pcl');

idx_use = sum(vertexColData')~=0;
rgb_data_cprb = vertexColData(idx_use,:);
cprb = knnsearch(scene.get_class_colormap,rgb_data_cprb)'-1;




if 0, %% run 3rd layer again! :)
    save_res_per_facade2file = 0;  %%% if to save results
    facade_ids_go = unique(scene.facade_id);  %%% ids of facedes
    facade_run_3Layer3D;
end
    





%--- can be always on -> this defines what to calculate from 3rd layer boxes   
store_bbox = 1;  %%% if I want to store all boxes (yes for export)
estiamte_door_box = 1; %%% always on!


%--- what to export
plot_facades_into_images_save_them_separate     = 0;   %%% plot matlab images 
plot_boxes_while_optimizing                     = 1;   %%% plot boxes before and after optimization :)
plot_facade_3d_boxes_matlab                     = 1;   %%% plot matlab images of 3d boxes results :)
export_result_to_full_scene_ply                 = 0;   %%% for andelos's evaluation

%--- now 3ds max exports! :)
save_separate_rgb_projected3dfacades            = 0;   %%% each facade with boxes for projections :)



%--- init some vars
cprb_new_objs = cprb;
if store_bbox, %%% if I may export to 3ds and want to have 
    pts = []; tri = []; tri_color = []; pts_facid=[]; tri_facid=[]; pts_ori=[]; pts_classid=[];tri_classid=[]; pts_oid=[]; tri_oid=[];
    cmap = scene.get_class_colormap();cmap = cmap(2:end,:);     
end
labeling_cprb_doors = [];
if estiamte_door_box, %%% door needs probability map, not just labeling...
    labeling_cprb_doors = scene.create_oidx_from_lidx('cprb',cprb,'K',7,'cl',4,'max_objs',100);
    labeling_cprb_doors = labeling_cprb_doors.oindex;
    prob_classes = load(fullfile(path_cprb_probl,['3D_layer1-',num2str(2000),'-',num2str(200),'.mat']));
    prob_classes = prob_classes.dt(4:4+numel(scene.get_class_names),:);
end






%==========================================================================
%------- Juhuuu!!  now project boxes to pcl and update the probm.
%        & and estimate boxes during that..
%--------------------------------------------------------------------------
time_cumsum_3rd_end = 0;
for facade_id = unique(scene.facade_id),
    if facade_id < 2,
        continue;
    end
    %--- read boxes
    idx_in_facade = find(scene.facade_id==facade_id);
    fprintf('    facade id=%i, #pts=%.1fK,  ',facade_id,length(idx_in_facade)/1e3);
    path_data = scene.get_adr('wa_bboxes',input_type_into_3rd,facade_id);
    if ~exist(path_data,'file');
        fprintf('Does not exist!\n');
        continue;
    end
    t = dir(path_data);
    fprintf('    ...bbox file %s \n         created at  %s\n',path_data,t.date);
    load(path_data);  %%% read normals, bbox, bbox_ga and bbox_new
    %--- read grav. vector
    grav_vec = load([path_grav_vec,'/monge428New_fullRes_split_',num2str(facade_id),'_plane.mat']);
    grav_vec = grav_vec.g';
   
    tic;
    
    %---- project optimization to facade's labeling -------------------
    [pts_facade,pts_mean] = project_in3d_pts_to_planes_normal( scene.pts(:,idx_in_facade) , nrm , [] , grav_vec);
    fprintf('         bbox %i->%i->%i\n',length(bbox.id),length(bbox_ga.id),length(bbox_new.id));
    
    %--- Now, lets do something with boxes... firstly reset to wal/sky/balc..
    tbox = facade_estimate_wall_bboxes(bbox_new,cprb,idx_in_facade,pts_facade,pts_in_bbox,labeling_cprb_doors,prob_classes);
    cprb_new_objs(idx_in_facade) = 2; %%% set everything to wall
    %---- now project classes that are not win. or balc.
    for co = [6 5 7 4];
        ids_local  = cprb(idx_in_facade)==co;  %%% idxs in facade
        if sum(ids_local)==0; continue; end;
        tmp_corners = tbox.corners(:,tbox.class==co);
        if isempty(tmp_corners), continue; end;
        idx_pts_in_corners = pts_in_bbox(pts_facade,tmp_corners);
        cprb_new_objs(idx_in_facade(idx_pts_in_corners)) = co;
    end
    
    %--- now project the value of each box (win+blac)
    for b=1:length(bbox_new.class),
        %--- pts in the bbox
        idx_pts_in_box_facade = pts_in_bbox(pts_facade,bbox_new.corners(:,b));
        idx_in_box = logical(scene.pts(1,:)*0);
        %--- assign value
        idx_in_box(idx_in_facade) = idx_pts_in_box_facade;
        cprb_new_objs(idx_in_box) = bbox_new.class(b);
    end
    time_cumsum_3rd_end =  time_cumsum_3rd_end +toc;
    
    %--- calculation done, now just processing the data for
    %       visualization that is needed....
    
    
    %--- if you want to store all boxes together for furter visualization in 3ds max.
    if store_bbox, %     pts=[]; tri=[]; tri_color=[]; pts_facid=[]; tri_facid=[]; pts_ori=[];
        pts_move_mean = mean(scene.pts(:,idx_in_facade),2);
        move_pts_to_pos = @(pts_mean,pts,II) bsxfun(@plus,[pts_mean],[II(1,:)*pts ; II(2,:)*pts ; II(3,:)*pts]);
        get_wall        = @(pts) [min(pts(1:2,:),[],2) ; mean(pts(3,:))+0.01 ; max(pts(1:2,:),[],2) ; mean(pts(3,:))-0.01];
        [v1,v2]       = get_coorframe_given_normal(nrm);
        II            = inv([v1';v2';nrm']);  %%% calcualte back-projection from single facade to scene
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
    cam_set = @() eval(' axis equal off; campos([0 -1 3]); camup([-1 0 0]); camzoom(0.8); camproj(''orthographic''); set(gcf, ''Position'', [400 400 800 600]); set(gcf, ''Color'', [1 1 1]); zoom(1.2);');
    if plot_boxes_while_optimizing
        path2save = '../export/pics_optim_wa_boxes/';
        cmap = scene.get_class_colormap();cmap = cmap(2:end,:);
        try; close all; end;
        % hf=figure(); honza_scatter(pts_facade,pts_facade(1,:)*0+20,cmap(cprb(idx_in_facade),:)/255,'filled');  set(hf,'Position',[0,0,600,700]); set(gcf, 'Color', [1 1 1]); zoom(1.2); axis off equal; view(90,90);
        % im = screencapture(hf); delete(hf); imwrite(imcrop(im,[10 80 size(im,2)-20 size(im,1)-120]),[path2save,num2str(facade_id),'_',input_type_into_3rd,'_0prob.jpg']);
        try delete(hf); end;  hf=figure(); for cnt=1:length(bbox.class), col=[1 0 0];if bbox.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.35); hold on; end; cam_set();
        export_fig([path2save,num2str(facade_id),'_',input_type_into_3rd,'_1initial.jpg']);
        try delete(hf); end;   hf=figure(); for cnt=1:length(bbox_ga.class), col=[1 0 0];if bbox_ga.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox_ga.corners([3 6],cnt),bbox_ga.corners([2 5],cnt),bbox_ga.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.35); hold on; end;  cam_set();
        export_fig([path2save,num2str(facade_id),'_',input_type_into_3rd,'_2ga.jpg']);
        try delete(hf); end;   hf=figure(); for cnt=1:length(bbox_new.class), col=[1 0 0];if bbox_new.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox_new.corners([3 6],cnt),bbox_new.corners([2 5],cnt),bbox_new.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.35); hold on; end; cam_set();
        export_fig([path2save,num2str(facade_id),'_',input_type_into_3rd,'_3optim.jpg']);
        try delete(hf); end; close all;
        hf = figure(32); for cnt=1:length(tbox.class); col=cmap(tbox.class(cnt),:)/255; honza_plot_3d_cube(tbox.corners([3 6],cnt),tbox.corners([2 5],cnt),tbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',1); hold on; end; alpha(0.8); cam_set();
        export_fig([path2save,num2str(facade_id),'_',input_type_into_3rd,'_4fac.jpg']); delete(hf);
        fprintf('         bbox saved to: %s: %s_%i...\n',path2save,input_type_into_3rd,facade_id);
    end
end




%--- separate facades for 3ds max as BOXES with RGB projection :)
if save_separate_rgb_projected3dfacades,
    path_3ds_export_fac = '../export/separ_facades/';
    for facade_id = unique(pts_facid),
        
        im_path = [path_images_source,'monge428New_fullRes_split_',num2str(facade_id),'_ortho_colors.png'];
        %im_path = ['/esat/sadr/amartino/monge428New/data/work/pcl/split/monge428New_fullRes_layer1+3D_3DCRF/monge428New_fullRes_split_',num2str(facade_id),'_ortho_labeling.png'];
        path_fac_img = sprintf('%s/%i.png',path_3ds_export_fac,facade_id);
        path_fac_obj = sprintf('%s/%i',path_3ds_export_fac,facade_id);
        
        im = imread(im_path);   %% read andelos image
        im = imrotate(im,90);
        imwrite(im , path_fac_img);
        fprintf('   ...image copy %s -> %s\n',im_path,path_fac_img);
        
        %--- now 3D :) ----------------------------------------------------  
        %--- read angelo's datra to images and find corners
        dt_plane_andelo = load([path_im_data_file,'/monge428New_fullRes_split_',num2str(facade_id),'_plane.mat']);
        im_corners = dt_plane_andelo.plane.b;
        load(scene.get_adr('wa_bboxes',input_type_into_3rd,facade_id));  %%% read boxes for actual facade...
        [~,pts_mean]      = project_in3d_pts_to_planes_normal( scene.pts(:,scene.facade_id==facade_id) , nrm , [] , grav_vec);
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
        disp(['   ...bin boxes saved for 3ds to: ',path_fac_obj,'_wallWin.boxesbin']);
        
        %--- save blaconies
        id2save = 3; facade_save_facades_boxes_for_3ds(pts_ori , tri , pts_facid==facade_id & pts_classid==id2save ,...
            tri_facid==facade_id & tri_classid==id2save , im_corners_proj , [path_fac_obj,'_balc.obj'],move_in_x);
    end
end


% 
% 
% %--- project to full pcl (with background)  and save to andelo :)
% if export_result_to_full_scene_ply
%     %--- project to full pcl
%     if ~exist('full_pcl','var') || isempty(full_pcl) || ~strcmp(scene.read_file_name{1},full_pcl.read_file_name{1}),  %%% if does not exist, is empty, or nto the same as current :)  
%         disp('  ...reading full res PLC...');
%         full_pcl = SScene_An(path_mat_data_dir,postfix_path_data_orig);  %%% this is where clicking result saved
%         full_pcl = full_pcl.read_mat_data();
%         i_pcl2full = knnsearch(scene.pts',full_pcl.pts');
%     else
%         disp('   ...full-pcl exists. Ahhh, thanks god I dont have to read it again :)');
%     end
%     %cprb_full = full_pcl.lindex*0;    
%     cprb_full = cprb_new_objs(i_pcl2full);
%     cprb_full(full_pcl.flag~=2)=0;
%     %--- visualize
%     if 0,
%         cmap = full_pcl.get_class_colormap();  %cmap = cmap(2:end,:);
%         colors_show = uint8([ (cmap(cprb_full+1,:)+full_pcl.rgb'*2)/3]);
%         full_pcl.plot_opengl_scene([full_pcl.lindex',cprb_full',cprb(i_pcl2full)'],'Q_rgb',colors_show);
%     end
%     %--- save to andelo
%     cmap = full_pcl.get_class_colormap();
%     path2save_full_2andelo =  [ADD.data.dtsol,'/2andelo/full_pcl_labeling_',input_type_into_3rd,'_3D3rdLayer.ply'];
%     ExportMesh(path2save_full_2andelo , full_pcl.pts',[],cmap(cprb_full+1,:),[],[]);
%     disp(['   ...result saved as pcl in the full-pcl.ply format for evaluation. path=',path2save_full_2andelo]);
% end
% 




    