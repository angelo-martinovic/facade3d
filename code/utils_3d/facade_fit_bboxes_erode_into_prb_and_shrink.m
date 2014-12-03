


%function bbox = facade_fit_bboxes_erode_into_prb_and_shrink(pts_facade,labeling_cprb,idx_in_facade,clIdxs,pts_in_bbox)
function bbox = facade_fit_bboxes_erode_into_prb_and_shrink(pts_facade,cprb,idx_in_facade,clIdxs,pts_in_bbox)

%--- check dataset for resolution...
global CFG
%dataset_n_for_closing = [4 10];
dataset_n_for_closing = [2 5];


%--- init pars 
% median_size = [0.1995 0 0.2100;0.1853  0  0.2217];
min_box_sizeSq = 0.004;%.0054;
max_box_sizeSq = 1.62;%9;
dist_trshld_overlap = 0.001;
bbox.center  = [];  bbox.corners = [];  bbox.class   = []; bbox.id = [];

compute_symmetry = 1;

%--- find symmetry plane
if compute_symmetry,
    %pts_facade_wins = pts_facade(:,labeling_cprb(idx_in_facade,1)>0);
    pts_facade_wins = pts_facade(:,cprb(idx_in_facade)==1);
    symmetr_score = 1e9;
    for tsymmetr = [-.3:.02:.3];
        idx_higher  = pts_facade_wins(2,:)>tsymmetr;
        idx_lower  = pts_facade_wins(2,:)<tsymmetr;
        pts1 = pts_facade_wins(:,idx_higher);
        pts2 = pts_facade_wins(:,idx_lower);
        pts1(2,:) = tsymmetr-(pts1(2,:)-tsymmetr);
        [~,d1] = knnsearch(pts1',pts2');
        [~,d2] = knnsearch(pts2',pts1');
        if (sum(d1)+sum(d2))<symmetr_score,
            symmetr_score = sum(d1)+sum(d2);
            symmetry = tsymmetr;
        end
    end
    % mukta_plot(pts_facade_wins(:,idx_higher),'.r'); hold; mukta_plot(pts_facade_wins(:,idx_lower),'.b'); mukta_plot([-2,symmetry,0;2,symmetry,0]','-k');   
end

for c=clIdxs;

    %---- start with erosion
    %labeling_in_facade_c{1} = labeling_cprb(idx_in_facade,c)'>0;
    labeling_in_facade_c{1} = cprb(idx_in_facade)==c;
    %--- do closing :)
    i_nn = knnsearch(pts_facade',pts_facade','k',10);
    heigh_shrink_par = .1;
    i_nn_heigh_shrink = knnsearch(bsxfun(@times , pts_facade , [1 heigh_shrink_par 1]')',bsxfun(@times , pts_facade , [1 heigh_shrink_par 1]')','k',6);
    labeling_in_facade_c{2} = ~facade_3d_morph_operations('closing',~labeling_in_facade_c{1},i_nn,dataset_n_for_closing(1));
    labeling_in_facade_c{3} = facade_3d_morph_operations('closing',labeling_in_facade_c{1},i_nn_heigh_shrink,dataset_n_for_closing(2));
    if 0,
        a=1; figure(a); honza_scatter(pts_facade,pts_facade(1,:)*0+15,labeling_in_facade_c{a},'filled'); set(gcf, 'Color', [1 1 1]); axis equal off; view(90,90);
        a=2; figure(a); honza_scatter(pts_facade,pts_facade(1,:)*0+15,labeling_in_facade_c{a},'filled'); set(gcf, 'Color', [1 1 1]); axis equal off; view(90,90);
        a=3; figure(a); honza_scatter(pts_facade,pts_facade(1,:)*0+15,labeling_in_facade_c{a},'filled'); set(gcf, 'Color', [1 1 1]); axis equal off; view(90,90);
    end
    for erode_level = 1:length(labeling_in_facade_c),
         % labels_act = find_connected_componets_in_3Dpcl(pts_facade,labeling_in_facade_c{erode_level},'K',15,'cl',1,'max_objs',length(unique(labeling_cprb))*2); %%% class is alwys one becuase we do in logical for only c :)
         labels_act = find_connected_componets_in_3Dpcl(pts_facade,labeling_in_facade_c{erode_level},'edges',i_nn','cl',1,'max_objs',300); %%% class is alwys one becuase we do in logical for only c :)
         if 0,
             figure(223); honza_scatter(pts_facade,pts_facade(1,:)*0+25,labeling_in_facade_c{erode_level},'filled'); set(gcf, 'Color', [1 1 1]); axis equal off; view(90,90);
             figure(224); honza_scatter(pts_facade,pts_facade(1,:)*0+25,labels_act,'filled'); set(gcf, 'Color', [1 1 1]); axis equal off; view(90,90);
         end
        for id_component = unique(labels_act),
            if id_component==0, continue; end;
            ids_component_local  = labels_act'==id_component;  %%% idxs in component!!!
            if sum(ids_component_local)<10, continue; end; %%% not enough points in the component...
            %--- shrink them
            shrink_vects = [-1 -1 0 1 1 0];
            corners_limit_max    = [max(pts_facade(:,ids_component_local),[],2) ; min(pts_facade(:,ids_component_local),[],2)];
            num_pts_orig_inside  = sum(pts_in_bbox(pts_facade(:,ids_component_local),corners_limit_max));
            %--- before I was trunning shrink for diff.. directions...
            for shrink_dim = 1:size(shrink_vects,1),
                tmp_corners = corners_limit_max;
%                while num_pts_orig_inside*.92 < sum(pts_in_bbox(pts_facade(:,ids_component_local),tmp_corners)),
%                    tmp_corners = tmp_corners + shrink_vects(shrink_dim,:)'*.001;
%                end
%                 %--- check if it already exist (so there is a box with overlap...
                add_box_flag = 1;
%                 if length(bbox.corners)==0,  %%% actually we have nothiong to compare :)
%                     add_box_flag = 1;
%                 else
%                     [~,d1] = tmp_corners([1 2 4 5],:)' - bbox.corners(1,:)'
%                     [~,d2] = knnsearch(tmp_corners(4:6,:)',bbox.corners(4:6,:)');
%                     add_box_flag = sum(max(d1,d2)<dist_trshld_overlap)==0;
%                 end
                %--- check if the box isnt smaller than soem min size :)
                %if sum((tmp_corners(1)-tmp_:2)-tmp_corners(4:5)).^2)<min_box_sizeSq,
                if abs( (tmp_corners(1)-tmp_corners(4))*(tmp_corners(2)-tmp_corners(5)) )<min_box_sizeSq,
                    add_box_flag = 0;
                end
                %--- if so big... /:
                if abs( (tmp_corners(1)-tmp_corners(4))*(tmp_corners(2)-tmp_corners(5)) )>max_box_sizeSq
                   add_box_flag = 0;
                end 
                local_labeling  = labeling_in_facade_c{1}(ids_component_local);
                if sum(local_labeling(  pts_in_bbox(pts_facade(:,ids_component_local),tmp_corners)   ))==0
                    add_box_flag = 0;
                end
                if add_box_flag,  %%% no same data
                    %--- save them
                    tmp_center = mean(pts_facade(:,ids_component_local),2);
                    bbox.center  = [bbox.center , mean(pts_facade(:,ids_component_local),2)];
                    bbox.corners = [bbox.corners , tmp_corners];
                    bbox.class   = [bbox.class , c];
                    bbox.id      = [bbox.id , id_component];
                    if compute_symmetry,
                        %--- also mirror it :)
                    %    mukta_plot(tmp_corners(1:3,:),'.r'); hold on; mukta_plot(tmp_corners(4:6,:),'.b');
                        increase_or_decr   = (tmp_corners([2 5],:)<symmetry)*2-1;      %%% get -1 if bigger than symmetry line and 1 for smaller
                        dist_to_s_center   = abs(tmp_corners([2 5])-symmetry);         %%% distance to the symmwetry line
                        tmp_corners([2 5]) = symmetry+dist_to_s_center([2 1]).*increase_or_decr([2 1]); %%% left becomes right
                        %mukta_plot(tmp_corners(1:3,:),'xr'); mukta_plot(tmp_corners(4:6,:),'xb');  mukta_plot([-1,symmetry,0]','xk','LineWidth',2);xlabel('x'); ylabel('y'); zlabel('z'); axis on; grid on; hold on;        
                        %--- save it :)
                        bbox.center  = [bbox.center , mean(tmp_corners([1 2 3; 4 5 6]))'];
                        bbox.corners = [bbox.corners , tmp_corners];
                        bbox.class   = [bbox.class , c];
                        bbox.id      = [bbox.id , id_component];
                    end
                end
            end
        end
        if 0
            honza_scatter(pts_facade,pts_facade(1,:)*0+10,labeling_cprb(idx_in_facade),'filled'); hold on; 
            for cnt=1:size(bbox.corners,2), honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor','r','FaceAlpha',.5);hold on;end;    set(gcf, 'Color', [1 1 1]); axis equal off; view(90,90);
        end
    end
end


if 1,
    %-- andelos erasing duplicats
    pool = [bbox.corners([5 4 2 1],:) ; bbox.class];

    for c=unique(bbox.class),
        median_size(1,c) = median(abs(bbox.corners(1,bbox.class==c)-bbox.corners(4,bbox.class==c)))/2;
        median_size(2,c) = median(abs(bbox.corners(2,bbox.class==c)-bbox.corners(5,bbox.class==c)))/2;
    end
    
    
    hyperParameters.win_ddw = median_size(2,1);
    hyperParameters.win_ddh = median_size(1,1);
    hyperParameters.balc_ddw = median_size(2,3);
    hyperParameters.balc_ddh = median_size(1,3);
    
    
    [~,removeFlags] = RemoveDuplicates(pool,hyperParameters);
    
    bbox.center  = bbox.center(:,~removeFlags);
    bbox.corners = bbox.corners(:,~removeFlags);
    bbox.class   = bbox.class(~removeFlags);
    bbox.id      = bbox.id(~removeFlags);
end

end









function [current_boxes,removeFlags] = RemoveDuplicates(initElements,hyperParams)
%     figure(100);imagesc(LabelingFromBoxes(initElements,false,hyperParams));axis equal;
removeFlags = zeros(size(initElements,2),1);
elemTypes = unique(initElements(5,:));
for i=elemTypes
    indices = find(initElements(5,:)==i);
    elems = initElements(:,indices)';
    
    dmats = cell(4,1);
    for j=1:4
        %             elems = elems(:,1:4);
        
        % Find distances between pairs
        D = pdist(elems(:,j),@(x,y)BoxDistance(x,y,i,j));
        dmats{j} = squareform(D)==0;
    end
    dmat = ~(dmats{1} & dmats{2} & dmats{3} & dmats{4});
    
    % Find the ones that are zero
    inds = find(dmat==0 & triu(ones(size(dmat)),1)==1);
    
    % Remove the duplicate elements
    [~,jj]=ind2sub(size(dmat),inds);
    removeFlags(indices(jj)) = 1;
end

current_boxes = initElements(:,~removeFlags);
%     figure(101);imagesc(LabelingFromBoxes(current_boxes,false,hyperParams));axis equal;

    function d = BoxDistance(XI,XJ,objectType,direction)
        % XI is a row
        % XJ can be multiple rows
        absdist = abs(bsxfun(@minus,XI,XJ));
        if objectType==1
            ddw = hyperParams.win_ddw;
            ddh = hyperParams.win_ddh;
        else
            ddw = hyperParams.balc_ddw;
            ddh = hyperParams.balc_ddh;
        end
        
        if direction==1 || direction==3
            absdist(absdist<ddw)=0;
        else
            absdist(absdist<ddh)=0;
        end
        
        d = absdist;
        %         d = sum(absdist,2);
    end

end




