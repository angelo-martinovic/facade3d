



function bbox = facade_estimate_wall_bboxes(bbox,cprb,idx_in_facade,pts_facade,pts_in_bbox,labeling_cprb_doors,prob_classes)

%get_wall        = @(pts) [min(pts(1:2,:),[],2) ; mean(pts(3,:))+0.01 ; max(pts(1:2,:),[],2) ; mean(pts(3,:))-0.01];


%--- ioptim lines that splits facade into shop/wall/roof..
cprb2find_lines = cprb(idx_in_facade);
pts2find_lines  = pts_facade;
[line_floor line_top line_shopeWall line_wallRoof line_roofSky] = get_separ_lines(cprb2find_lines,pts2find_lines);

line_separ = [line_floor line_shopeWall line_wallRoof line_roofSky line_top];

%--- pllot also roof etc..
map_ln2class = [0 2 0 0 3 4 1];  %%% i.e. 7-th shop it is first in line_separ
for co = [2 5 6 7];  %%% [2 5 6 7]    [7 2 5 6];
    ids_local  = cprb(idx_in_facade)==co;  %%% idxs in facade
    if sum(ids_local)==0,
        continue;
    end
    tmp_corners = [line_separ(map_ln2class(co)) ; max(pts_facade(2:3,ids_local),[],2) ; line_separ(map_ln2class(co)+1) ; min(pts_facade(2:3,ids_local),[],2)];
    bbox.class    = [bbox.class co];
    bbox.corners  = [bbox.corners tmp_corners];
end
%--- set the wall's depth
bbox.corners([3,6],bbox.class==2) =  [mean(pts_facade(3,:))+0.01 ; mean(pts_facade(3,:))-0.15];

%--- move window/balc above wall
bbox.corners(3,bbox.class==1) = bbox.corners(3,bbox.class==2)+.04;
bbox.corners(6,bbox.class==1) = bbox.corners(3,bbox.class==2)-0.08;
bbox.corners(3,bbox.class==3) = bbox.corners(3,bbox.class==2)+.06;
bbox.corners(6,bbox.class==3) = bbox.corners(6,bbox.class==2)-.09;

%-- move shop too
bbox.corners(3,bbox.class==7) = bbox.corners(3,bbox.class==2)+.1;
bbox.corners(6,bbox.class==7) = bbox.corners(6,bbox.class==2);

%-- move roof as well
bbox.corners(3,bbox.class==5) = bbox.corners(3,bbox.class==2)-.05;
bbox.corners(6,bbox.class==5) = bbox.corners(6,bbox.class==6);

%--- and also sky according to roof...
if sum(bbox.class==5)>0,
    bbox.corners(3,bbox.class==6) = bbox.corners(6,bbox.class==5)+0.04;
    bbox.corners(6,bbox.class==6) = bbox.corners(6,bbox.class==5)-0.04;
end


%--- add door to place that best fits probability map
if exist('labeling_cprb_doors','var') && ~isempty(labeling_cprb_doors),
    median_size =[ 0.2066         0    0.1353 ;  0.1827         0    0.1657];   %%% this is from trainig data :)
    bbox_shop_corners = bbox.corners(:,bbox.class==7);
    doors_labels_local = labeling_cprb_doors(idx_in_facade);
    prob_classes_local = prob_classes(:,idx_in_facade);
    if sum(doors_labels_local)==0,
        [~,i_max_door] = max(prob_classes_local(4,:));
        doors_labels_local(i_max_door) = max(doors_labels_local)+1;
    end
    
    door_corners = [];
    door_scores = [];
    for d=unique(doors_labels_local)
        if d==0,
            continue;
        end
        boundsY = [max(pts_facade,[],2) ; min(pts_facade,[],2)];
        boundsY = boundsY([5 2]);
        
        tmp_corners = [max(pts_facade(:,doors_labels_local==d),[],2) ; min(pts_facade(:,doors_labels_local==d),[],2)];
%         tmp_corners = [max(pts_facade,[],2) ; min(pts_facade,[],2)];
        %--- optimize
        tmp_corners(1) = line_floor;
        tmp_cor = tmp_corners;
        tmp_center = (tmp_corners(1:3)+tmp_corners(4:6))/2;
        score = -5e6;
%         for pos_y = -.3+tmp_center(2):.1:.3+tmp_center(2)  %%% its horiziontal postion
         for width = median_size(2,1)*1.5:median_size(1,1)*.2:median_size(2,1)*2.5
            for pos_y = boundsY(1)+width/2:median_size(1,1)*.2:boundsY(2)-width/2
                for height = median_size(1,1)*1.5:median_size(1,1)*.5:median_size(1,1)*4
                    
                    tmp_cor(4) = line_floor-height; %%% upper
                    tmp_cor([2 5]) = [width/2 -width/2]+pos_y;
                    
                    idx_pts_in_box_      = logical(pts_in_bbox(pts_facade,tmp_cor));
                    t_score              = sum(prob_classes_local(4,idx_pts_in_box_))/sum(idx_pts_in_box_);
                    if t_score>score
                        if(abs(tmp_cor(2)-tmp_cor(5))<.05)
                            sdsdghgh
                        end
                        score = t_score;
                        tmp_corners = tmp_cor;
                    end
                end
            end
        end
        if ~isempty(tmp_corners)
            honza_scatter(pts_facade,pts_facade(1,:)*0+15,prob_classes_local(4,:),'filled'); honza_plot_3d_cube(tmp_corners([3 6]),tmp_corners([2 5]),tmp_corners([1 4]),'FaceColor','r','FaceAlpha',.8); view(90,90);    
            title(num2str(score));
            door_corners = [door_corners tmp_corners];
            door_scores  = [door_scores score];
        end
    end
        
% %      honza_scatter(pts_facade,pts_facade(1,:)*0+15,prob_classes_local(4,:),'filled'); honza_plot_3d_cube(tmp_corners([3 6]),tmp_corners([2 5]),tmp_corners([1 4]),'FaceColor','r','FaceAlpha',.8); view(90,90);    
%         honza_scatter(pts_facade,pts_facade(1,:)*0+15,prob_classes_local(4,:),'filled'); honza_plot_3d_cube(tmp_corners([3 6]),tmp_corners([2 5]),tmp_corners([1 4]),'FaceColor','r','FaceAlpha',.8); view(90,90);    
%         
%         if ~isempty(tmp_corners)
%             door_corners = [door_corners tmp_corners];
%             door_scores  = [door_scores score];
%             bbox.corners = [bbox.corners [tmp_corners(1:2); bbox_shop_corners(3)+0.1;  tmp_corners(4:5); bbox_shop_corners(6)-0.1]];
%             bbox.class = [bbox.class 4];
% %             cnt=size(door_corners,2);
% %             honza_plot_3d_cube(door_corners([3 6],cnt),door_corners([2 5],cnt),door_corners([1 4],cnt),'FaceColor','r','FaceAlpha',1); hold on;
% %             text(door_corners(1,cnt),door_corners(2,cnt),door_corners(3,cnt),num2str(cnt));
%         end
        
    %--- test overlap
%     pool = [door_corners([5 4 2 1],:)];
%     for d=1:length(door_scores),
%         bbox1 = pool(:,d);
%         use_d = 1;
%         for d_test = 1:length(door_scores)
%             if d==d_test, continue; end;
%             bbox2 = pool(:,d_test);
%             max_x1 = max(bbox1(1), bbox2(1));
%             min_x2 = min(bbox1(3), bbox2(3));
%             overlap_x = (min_x2 - max_x1);
%             overlap_x = overlap_x > .5*(bbox2(3)-bbox2(1));
%             overlap = ~(bbox2(3) < bbox1(1) | bbox2(1) > bbox1(3)  );
%             if overlap,
%                 if door_scores(d)<door_scores(d_test),
%                     use_d = 0;
%                 end
%             end   
%         end    
%         if use_d
%             bbox.corners = [bbox.corners [door_corners(1:2,d); bbox_shop_corners(3)+0.1;  door_corners(4:5,d); bbox_shop_corners(6)-0.1]];
%             bbox.class = [bbox.class 4];
%         end
%     end
    [~,d] = max(door_scores);
    bbox.corners = [bbox.corners [door_corners(1:2,d); bbox_shop_corners(3)+0.1;  door_corners(4:5,d); bbox_shop_corners(6)-0.1]];
    bbox.class = [bbox.class 4];
end

end








%%  get lines that sepraates shop from wall from roof...
function [line_floor, line_top, line_shopeWall, line_wallRoof, line_roofSky] =  get_separ_lines (cprb_facade,pts_facade)


%--- window->wall... blac->wall
cprb_facade(cprb_facade==1) = 2;
cprb_facade(cprb_facade==3) = 2;
cprb_facade(cprb_facade==4) = 7;

%--- init
dim = 1;
line_floor = max(pts_facade(dim,:));
line_top   = min(pts_facade(dim,:));
line_shopeWall    = mean([min(pts_facade(dim,cprb_facade==7)),max(pts_facade(dim,cprb_facade==2))]);
line_wallRoof     = min(line_shopeWall , mean([min(pts_facade(dim,cprb_facade==2)),max(pts_facade(dim,cprb_facade==5))]));
line_roofSky      = min(line_wallRoof , mean([min(pts_facade(dim,cprb_facade==5)),max(pts_facade(dim,cprb_facade==6))]));


input_data = [line_shopeWall,line_wallRoof,line_roofSky];

step_ = .12;
limit_ = .7;
energy = energy_(input_data,cprb_facade,pts_facade,line_floor,line_top);
for sw = line_shopeWall-limit_:step_:line_shopeWall+limit_;
    for wr = line_wallRoof-limit_:step_:line_wallRoof+limit_;
        for rs = line_roofSky-limit_:step_:line_roofSky+limit_;
            tmp = energy_([sw wr rs],cprb_facade,pts_facade,line_floor,line_top);
            if tmp<energy, energy = tmp; line_shopeWall = sw; line_wallRoof = wr; line_roofSky = rs;  end
        end;
    end;
end;
step_ = .045;
limit_ = .12;
for sw = line_shopeWall-limit_:step_:line_shopeWall+limit_,
    for wr = line_wallRoof-limit_:step_:line_wallRoof+limit_,
        for rs = line_roofSky-limit_:step_:line_roofSky+limit_,
            tmp = energy_([sw wr rs],cprb_facade,pts_facade,line_floor,line_top);
            if tmp<energy, energy = tmp; line_shopeWall = sw; line_wallRoof = wr; line_roofSky = rs;  end
        end;
    end;
end;






end



%% energy of the current sepration lines
function energy = energy_(lines,cprb_facade,pts_facade,ln_floor,ln_top)
pts_dim        = pts_facade(1,:);
cprb_new       = cprb_facade*0+6; %%% everything sky
cprb_new(pts_dim>lines(3)) = 5;
cprb_new(pts_dim>lines(2)) = 2;
cprb_new(pts_dim>lines(1)) = 7;
norm_val = [ 0 abs(lines(1)-lines(2)) 0 0 abs(lines(2)-lines(3)) abs(ln_top-lines(3)) abs(ln_floor-lines(1))];



energy = 0;
for class = [2 5 6 7],%1:unique(cprb_facade),
    energy = energy - sum(cprb_new(cprb_new==class)==cprb_facade(cprb_new==class));%/sum(cprb_facade==class);   %/norm_val(class)
end

    

if isnan(energy)
    energy = 1e6;
end

end
