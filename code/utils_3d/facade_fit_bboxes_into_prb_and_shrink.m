


function bbox = facade_fit_bboxes_into_prb_and_shrink(pts_facade,labeling_cprb,idx_in_facade,clIdxs,similar_principles,pts_in_bbox)


do_shrink_multi = 1;

bbox.center  = [];  bbox.corners = [];  bbox.class   = []; bbox.id = [];
for c=clIdxs;
    if similar_principles, ism_n = IISM_facade; ism_n.ism_grid = 0;  end
    labeling_in_facade_c = labeling_cprb(idx_in_facade,c)';
    for id_component = unique(labeling_in_facade_c),
        if id_component==0, continue; end;
        ids_component_local  = labeling_in_facade_c'==id_component;  %%% idxs in facade
        if sum(ids_component_local)<20, continue; end; %%% not enough points in the comonent...
        %--- shrink them
        if do_shrink_multi,
             shrink_vects = [-1 0 0 0 0 0 ; 0 -1 0 0 0 0 ; 0 0 0 1 0 0 ; 0 0 0 0 1 0];
        else
             shrink_vects = [-1 -1 0 1 1 0];
        end
        corners_limit_max    = [max(pts_facade(:,ids_component_local),[],2) ; min(pts_facade(:,ids_component_local),[],2)];
        num_pts_orig_inside  = sum(pts_in_bbox(pts_facade(:,ids_component_local),corners_limit_max));
        for shrink_dim = 1:size(shrink_vects,1),
            tmp_corners = corners_limit_max;
            while num_pts_orig_inside*.8 < sum(pts_in_bbox(pts_facade(:,ids_component_local),tmp_corners)),
                tmp_corners = tmp_corners + shrink_vects(shrink_dim,:)'*.001;
            end
            %--- save them
            bbox.center  = [bbox.center , mean(pts_facade(:,ids_component_local),2)];
            bbox.corners = [bbox.corners , tmp_corners];
            bbox.class   = [bbox.class , c];
            bbox.id      = [bbox.id , id_component];
        end
        %--- copy shirinked vects
        if do_shrink_multi,
            idx_dim_shrink = [1 2 4 5];
            dim_shrink_in_bbox = length(bbox.id) - [3:-1:0];
            t = repmat(corners_limit_max,1,6+4+1);  %%% for all cpouples, tripels and one foursome :)
            ids_perm = [1 2 ; 1 3 ; 1 4 ; 2 3 ; 2 4 ; 3 4]';
            for cnt = 1:size(ids_perm,2),
                id_perm = ids_perm(:,cnt);
                t(idx_dim_shrink(id_perm),cnt) =  bbox.corners(  sub2ind( size(bbox.corners) , idx_dim_shrink(id_perm)' , dim_shrink_in_bbox(id_perm)' ) );
            end
            ids_perm = [1 2 3; 1 2 4; 1 3 4 ; 2 3 4]';
            for cnt = 1:size(ids_perm,2),
                id_perm = ids_perm(:,cnt);
                t(idx_dim_shrink(id_perm),cnt+6) =  bbox.corners(  sub2ind( size(bbox.corners) , idx_dim_shrink(id_perm)' , dim_shrink_in_bbox(id_perm)' ) );
            end
            id_perm = [1 2 3 4]';
            t(idx_dim_shrink(id_perm),end) = bbox.corners(  sub2ind( size(bbox.corners) , idx_dim_shrink(id_perm)' , dim_shrink_in_bbox(id_perm)' ) );
            bbox.center  = [bbox.center repmat(bbox.center(:,end),1,size(t,2))];
            bbox.corners = [bbox.corners t];
            bbox.class   = [bbox.class t(1,:)*0+c];
            bbox.id      = [bbox.id t(1,:)*0+id_component];
        end
%        for cnt=1:size(bbox.corners,2), honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor','r','FaceAlpha',.5);hold on;end
        if 0, %%% show points and fitted box
            honza_scatter(pts_facade,pts_facade(1,:)*0+1,cprb(idx_in_facade),'filled'); hold on; set(gcf, 'Color', [1 1 1]); axis equal off
            mukta_plot(pts_facade(:,ids_component_local),'.k');
            honza_plot_3d_cube(bbox.corners([3 6],end),bbox.corners([2 5],end),bbox.corners([1 4],end),'FaceColor','r','FaceAlpha',.5);
            mukta_plot(bbox.corners([1:3],end),'xb','LineWidth',3,'MarkerSize',20);
        end
        %--- if symmetry principles
        if similar_principles, %%% only if syummetry...
            ids_component_global = idx_in_facade(ids_component_local);   %%% idxs iin SScene
            ism_desc_type = 'rgb';
            zero_vec = find(ids_component_local)'*0;
            ism_n.vec2objCen     = [ism_n.vec2objCen bsxfun(@plus , -scene.pts(:,ids_component_global) , mean(scene.pts(:,ids_component_global),2))];
            ism_n.desc           = [ism_n.desc scene.(ism_desc_type)(:,ids_component_global)];
            ism_n.objSize        = [ism_n.objSize zero_vec+norm(bbox.corners(1:3,end)-bbox.corners(4:6,end))];
            ism_n.class          = [ism_n.class zero_vec+c];
            ism_n.modelID        = [ism_n.modelID zero_vec+id_component];
        end
    end
    if similar_principles, %%% only if symmetry...
        ism_n = ism_n.get_only_part_of_data(1:2:length(ism_n.modelID));
        [vari_multi_dim,par_wgh] = ism_n.get_par_voting(c);
        par_wgh.desc_stDev = par_wgh.desc_stDev*.7;
        [precomp_data(c),gd2{c}] = honza_seg_cast_votes_facade(ism_n,[],'scene_pt_used',idx_in_facade,'ism_nn_method','flann','desc_type',ism_desc_type,'par_wgh',par_wgh,'clIdx',c,'vari_multi_dim',vari_multi_dim,'delete_first_best_votes',1);
        
        Qprev = scene.desc(1,:)'*0+1;
        wgh  = Qprev(precomp_data(c).idx_vt2desc( precomp_data(c).ids(2:end,:) ));  %%% get desc_id of votes and their Q-value, so I can compute how I beliave they are correct
        vals{c} = sum(gd2{c}.*wgh)/size(wgh,1);%./sum(g);%/Nnn  %%% compute the value for every vote
        %                    [Q(:,c),~] = mex_ism_infer(double(vals{c}),int32(precomp_data(c).idx_vt2desc-1),int32(size(scene.desc,2)));
        fit{c} = votes_find_maxima_in_cont_space(precomp_data(c),vals{c},c,'store_occr_around',0,'rad',mean(ism_n.objSize),'tresh_filter_vals',.5);
        if 0,
            h=figure(3232);
            for cnt=1:length(bbox.class), col=[.5 0 0];if bbox.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.1,'EdgeColor','none'); hold on; end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]);
            % mukta_plot(pts_facade(:,labeling_cprb(idx_in_facade,c)>0),'.k'); hold on; axis equal;
            [~,v_id] = ismember(precomp_data(c).idx_vt2desc,idx_in_facade);
            %honza_plot_alpha_lines(pts_facade(:,v_id),project_in3d_pts_to_planes_normal(precomp_data(c).vt(1:3,:),nrm,pts_mean),'CData',[vals{c};vals{c}],'EdgeColor','flat');
            honza_plot_fuzzy_pc(project_in3d_pts_to_planes_normal(precomp_data(c).vt(1:3,:),nrm,pts_mean),'sc',.01,'EdgeAlpha',.1,'EdgeColor',[vals{c}].^5);
            honza_plot_2cols(project_in3d_pts_to_planes_normal(fit{c}.pos,nrm,pts_mean),{'xk',9,3},{'xw',5,1});
            view(50,50); set(gcf, 'Color', [1 1 1]); zoom(2)
            % export_fig(['~/public_html/_dataXX/2Angelo/bbox_fit_optimialize/_facade_votes=',num2str(facade_id),'_orig.jpg']);  disp('saved to ~/public_html/_dataXX/2Angelo/bbox_fit_optimialize/_faca');
            delete(h);
        end
    end
end
end