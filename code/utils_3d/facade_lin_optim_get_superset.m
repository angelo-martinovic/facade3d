

function bbox_new = facade_lin_optim_get_superset (bbox , cprb , median_size , varargin)
p = inputParser;
p.addOptional('UseParallel', 1);
p.addOptional('method', 'bins_compress');
p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;

dl = DispatchingLogger.getInstance();

%--- precomp that is usefull for both
n_class_pts_in_cprb = sum(cprb==3 | cprb==1);
pts_in_cprb         = length(cprb);


initial_res  = zeros(1,length(bbox.class));
% nvars       = size(initial_res,2);
nvars       = size(initial_res,2);
% lb_all      = initial_res*0;
% ub_all      = initial_res*0+1;
% IntCon      = 1:nvars;


%% create mat

% bineq = initial_res+1;
[Aineq,~,A] = create_ol_matrix([bbox.corners([4 5 1 2],:) ; bbox.class]);
if 0, %%% visulize Aineq
    id = 28;
    figure(23232);
    %A_norm = Aineq./max(Aineq(:));
    col=[1 0 0];
    for cnt=find(Aineq(id,:)),
        alph = .1;
        if cnt==id, alph=.6; end;
        honza_plot_3d_cube([1 1.01],bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',alph); hold on;
    end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]); zoom(1.2);
    
    for a=1:numel(bbox.class),
        mukta_plot(bbox.corners(1:3,:),'xr'); hold on;
        mukta_plot(bbox.corners(4:6,:),'og');
    end
    figure(324234); for cnt=1:length(bbox.class), col=[1 0 0];if bbox.class(cnt)==3,col=[.5 0 1];end;  text(bbox.center(1,cnt),bbox.center(2,cnt),bbox.center(3,cnt),num2str(cnt)); honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.2); hold on; end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]); zoom(1.2);
end


dl.Log(VerbosityLevel.Debug,...
    sprintf(' - - - Optimizing for best subset of %i boxes.\n',nvars));


     

%---- data term
%A = double(Aineq>0);
wdata_initial  = bbox.num_correct_pts_in - (bbox.num_pts_in-bbox.num_correct_pts_in);

% Linear term - normalize
wdata = wdata_initial;
% wdata = wdata - min(wdata);
% wdata = wdata / sum(wdata);
wdata = wdata / sum(abs(wdata));
% wdata = wdata / (max(wdata)-min(wdata));
wdata = -wdata;
    




pool = [bbox.corners([5 4 2 1],:) ; bbox.class];

%pool([3 4],:) = -pool([3 4],:);
hyperParameters.win_ddw = median_size(2,1)/2;
hyperParameters.win_ddh = median_size(1,1)/2;
hyperParameters.balc_ddw = median_size(2,3)/2;
hyperParameters.balc_ddh = median_size(1,3)/2;

%-- align + co-occur
C = zeros(nvars,nvars);
Q = zeros(nvars,nvars);
for i=1:nvars-1
    for j=i+1:nvars
        bbox1 = pool(:,i);
        bbox2 = pool(:,j);
        
        % Alignment term
        % For each object class separately
        if bbox1(5)==1 && bbox2(5)==1 || bbox1(5)==3 && bbox2(5)==3
            if bbox1(5)==1
                ddw =hyperParameters.win_ddw;
                ddh =hyperParameters.win_ddh;
            else
                ddw =hyperParameters.balc_ddw;
                ddh =hyperParameters.balc_ddh;
            end
            
            score = 0;
            if abs(bbox1(1)-bbox2(1))<ddw, score = score+1; end
            if abs(bbox1(3)-bbox2(3))<ddw, score = score+1; end
            if abs(bbox1(2)-bbox2(2))<ddh, score = score+1; end
            if abs(bbox1(4)-bbox2(4))<ddh, score = score+1; end
            
            Q(i,j) = -score;
            Q(j,i) = -score;
            
        end
    end
end

for i=1:nvars
    for j=1:nvars
        bbox1 = pool(:,i);
        bbox2 = pool(:,j);
        
        if (bbox1(5)==3 && bbox2(5)==1)
            C(i,i) = -1;
            % Check if balcony is under the window
            balconyHeight = hyperParameters.balc_ddh/.6;%.7; %bbox1(4)-bbox1(2);

            max_x1 = max(bbox1(1), bbox2(1));
            min_x2 = min(bbox1(3), bbox2(3));
            overlap_x = (min_x2 - max_x1);
            overlap_x = overlap_x > .5*(bbox2(3)-bbox2(1));
            overlap = ~(bbox2(3) < bbox1(1) | bbox2(1) > bbox1(3)  );
            touch = (abs(bbox1(2) - bbox2(4) ) < (balconyHeight* 0.7)) | (abs(bbox1(4) - bbox2(4) ) < (balconyHeight* 0.7));
            
            if (overlap && touch && overlap_x)
                C(i,j) = 1;
            end
        end
    end
end






 Q(1:size(Q,1)+1:end) = abs(sum(Q,2));
    
    
    
%% Solve the integer program
    
%     wdata = wdata / sum(wdata);
%     wdata = -wdata;
    b=1;
    
    wquad = Q(1:size(Q,1)+1:end);
    
    lambda = 10;
    Q = lambda * Q / sum(wquad);% sum(Q(1:size(Q,1)+1:end)) ;
    wquad = lambda * wquad/sum(wquad);
    
%     wdata_old = wdata;
    wdata = wdata-.5*wquad;
    
    cvx_begin quiet
        cvx_solver mosek
        cvx_solver_settings('MSK_IPAR_NUM_THREADS',1)   % 1 thread
        cvx_solver_settings('MSK_DPAR_MIO_MAX_TIME',20) % max 20 seconds
        variable x(nvars) binary
        minimize wdata'*x + .5*x.''*Q*x'
        subject to
                A*x <= b;
                C*x >= 0; 
                0 <= x <= 1;
    cvx_end
    best_boxes = pool(:,x>0.5);
    best_score = cvx_optval;


dl.Log(VerbosityLevel.Debug,...
    sprintf(' - - - - Score=%f. Done.\n',cvx_optval));
bbox_new.class   = bbox.class(x>.5);
bbox_new.id      = bbox.id(x>.5);
bbox_new.center  = bbox.center(:,x>.5);
bbox_new.corners = bbox.corners(:,x>.5);


%%
%  [optParams,~,~,~] = ga(@(x)facade_ga_energy_fce_andelos_mosac_optim(x,bbox,n_class_pts_in_cprb,median_size,pts_in_cprb) , nvars,double(Aineq>0),bineq,[],[],lb_all,ub_all,[],IntCon,optionsGA);




%s
if 0, %%% vis
    k = 10;
    szk = k*3+1+2+2;
    Q1 = Q;
    Q1(logical(eye(size(Q))))=0;
     figure(83); subplot(1,szk,1); imagesc(wdata'); title('data term');  set(gca,'xticklabel',[]);
       subplot(1,szk,3+(1:k)); imagesc(abs(Q1)); title('alignment');
       subplot(1,szk,3++k+(1:k)); imagesc(C); title('co-occurence');  set(gca,'yticklabel',[]);
       subplot(1,szk,3+2+k*2+(1:k));imagesc(A); title('overlap'); set(gca,'YAxisLocation','right');
       %colorbar; title('colorbar');
       set(gcf, 'Position', [0 0 1450 380]);
     if 0.
         figure(81); for cnt=1:length(bbox.class), col=[1 0 0];if bbox.class(cnt)==3,col=[.5 0 1];end;  text(bbox.center(1,cnt),bbox.center(2,cnt),bbox.center(3,cnt),num2str(cnt)); honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.2); hold on; end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]); zoom(1.2);
         figure(82); for cnt=1:length(bbox_new.class), col=[1 0 0];if bbox_new.class(cnt)==3,col=[.5 0 1];end;  honza_plot_3d_cube(bbox_new.corners([3 6],cnt),bbox_new.corners([2 5],cnt),bbox_new.corners([1 4],cnt),'FaceColor',col,'FaceAlpha',.2); hold on; end;  axis equal off; view(50,50); set(gcf, 'Color', [1 1 1]); zoom(1.2);
         for a=1:numel(bbox.class),
             mukta_plot(bbox_new.corners(1:3,:),'xr'); hold on;
             mukta_plot(bbox_new.corners(4:6,:),'og');
         end
     end
     

end

end
























%% Creates the overlap matrix
function [res_ol, res_minarea, A] = create_ol_matrix(all)
    
    res_ol = zeros(size(all,2));
    res_minarea = zeros(size(all,2));
    A = [];
    % For every pair of elements
    for i=1:size(all,2)
           a = all(:,i);
%            assert(i == a(7));
       for j=1:size(all,2)
           b = all(:,j);
%            assert(j == b(7));
           
           % If they are not the same class, skip
           if a(5)~=b(5)
               continue;
           end
           
           % Calculate the overlap of their BBs
           ol = bb_overlap(a,b);
           res_ol(i,j) = ol;
           
           if ol ==0
               continue;
           end
           
           constraintRow = zeros(1,size(all,2));
           constraintRow(i) = 1;
           constraintRow(j) = 1;
           A = [A; constraintRow ];
           
           % Get the area of the smaller element
           minarea = min(bb_area(a) ,bb_area(b));
           res_minarea(i,j) = minarea;
           
       end
    end
end
%%
function area = bb_overlap(a,b)
    area = 0;
    if (  (min(b(3), a(3))  > max(b(1),a(1))) && (min(b(4), a(4)) > max(b(2), a(2)))  ) 
        max_x1 = max(a(1), b(1));
        min_x2 = min(a(3), b(3));
        max_y1 = max(a(2), b(2));
        min_y2 = min(a(4), b(4));
        area = (min_x2 - max_x1) * (min_y2 -max_y1);
    end
   
end
function area = bb_area(a)
    area = (a(3) - a(1)) * (a(4)-a(2));
    
end


