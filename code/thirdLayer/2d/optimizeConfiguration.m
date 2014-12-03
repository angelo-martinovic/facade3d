function [best_score, best_boxes] = optimizeConfiguration(pool, outImg, hyperParameters)
    
%     id = 1:size(pool,2);
%     pool = [pool; id];
    nvars = size(pool,2);
    
    if 0,
        % Find overlapping elements
        [~, ~, A] = create_ol_matrix(pool);

        % Create the wdata matrix - unary term (penalty)
        
        wdata_initial = zeros(nvars,1);
        for i=1:nvars
            bbox = pool(:,i);

            gt = outImg(bbox(2):bbox(4),bbox(1):bbox(3));


            wdata_initial(i) =  sum(sum(gt==bbox(5))) - sum(sum(gt~=bbox(5)));
        end

        % Create the Q matrix - pairwise terms
        Q_initial = zeros(nvars,nvars);
        for i=1:nvars-1
            for j=i+1:nvars
                bbox1 = pool(:,i);
                bbox2 = pool(:,j);

                 % Alignment term
                 % For each object class separately
                 classes = sort([bbox1(5) bbox2(5)]);
                 if isequal(classes,[1 1]) || isequal(classes,[3 3]) || isequal(classes,[1 4])
                    if classes(1)==1
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

                    Q_initial(i,j) = -score;
                    Q_initial(j,i) = -score;

                 end
            end
        end


        % Create the C matrix - co-occurence term
        C = zeros(nvars,nvars);
        for i=1:nvars
            for j=1:nvars
                bbox1 = pool(:,i);
                bbox2 = pool(:,j);

                 if (bbox1(5)==3 && bbox2(5)==1)
                    C(i,i) = -1;
                    % Check if balcony is under the window
                    balconyHeight = bbox1(4)-bbox1(2);
                    overlap = ~(bbox2(3) < bbox1(1) | bbox2(1) > bbox1(3)  );
                    touch = (abs(bbox1(2) - bbox2(4) ) < (balconyHeight* 0.7)) | (abs(bbox1(4) - bbox2(4) ) < (balconyHeight* 0.7));

                    if (overlap && touch)
                        C(i,j) = 1;
                    end
                end
            end
        end


        %% IP
        % Solve the integer program

        % Linear term - normalize
        wdata = wdata_initial;
    %     wdata = wdata - min(wdata);
        wdata = wdata' / sum(abs(wdata));
        wdata = -wdata;

        % Quadratic term - make the Q matrix diagonally dominant and therefore
        % positive semidefinite
        Q = Q_initial;
        wquad = abs(sum(Q,2))';
        Q(1:size(Q,1)+1:end) = wquad;

        % How much alignment?
        lambda = 1;

        % Normalize
        Q = lambda * Q / sum(wquad);
        wquad = lambda * wquad/sum(wquad);

        % Normalization of unary term (removes the additional term from diagonal of Q)
        % NOTE: correct only if x is binary
        wdata = wdata-0.5*wquad;

        cvx_begin
            cvx_solver mosek
            cvx_solver_settings('MSK_IPAR_NUM_THREADS',1)   % 1 thread
            cvx_solver_settings('MSK_DPAR_MIO_MAX_TIME',10) % max 10 seconds
            variable x(nvars) binary
            minimize wdata'*x + 0.5*x.''*Q*x'
            subject to
                    A*x <= 1;
                    C*x >= 0;
                    0 <= x <= 1;
        cvx_end

        best_boxes = pool(:,x>0.5);
        best_score = cvx_optval;
    else

    %% GA 
        % Find overlapping elements
        [ol_matrix, ~]= create_ol_matrix(pool);

        % Add linear inequality constraints
        % Only one of multiple overlapping elements can be selected
        A = double(ol_matrix>0);
        b = ones(size(ol_matrix,1),1);

        % Energy to minimize
        f = @(x)TotalEnergy(x, pool, outImg, hyperParameters);

        
        % Integral constraints
        lb_all = zeros(nvars,1);
        ub_all = ones(nvars,1);
        IntCon = 1:nvars;
                 
        fprintf('nvars: %d\n',nvars);
        
        % GA optimization settings
        optionsGA = gaoptimset('Display','iter',...
            'Generations',hyperParameters.ga.nGenerations,...
            'StallGenLimit',hyperParameters.ga.stallGenLimit,...
            'PopulationSize',hyperParameters.ga.populationSize,...
            'UseParallel',hyperParameters.parallel);
        
        if hyperParameters.visualize
            optionsGA = gaoptimset(optionsGA,...
            'OutputFcns',{@(x,y,z) ShowLabeling(x,y,z, pool,hyperParameters)},...
            'PlotFcns',{@gaplotbestf, @gaplotdistance ,@gaplotrange, @gaplotstopping});
        end
        
        % Optimize if there are elements
        if nvars>0
            [optParams,~,~,~] = ga(f,nvars,A,b,[],[],lb_all,ub_all,[],IntCon,optionsGA);
        
            % Select only the optimal elements
            best_boxes =  pool(:,optParams==1);
        else
            best_boxes = pool;
        end
    
        % Score the best configuration
        best_score = doScoring(best_boxes,outImg,hyperParameters);
    end
end

function [state,options,optchanged]=ShowLabeling(options,state,flag,pool,hyperParameters)
    optchanged = [];
    
    % If the optimization is running
    if ~strcmp(flag,'init')
        % Get the best individual
        x = state.Population(find(state.Score==state.Best(end),1),:);
        
        % Select the elements
        tmp_boxes = [pool(:,x==1)];
        
        % Show the best labeling so far
        figure(100);imagesc(LabelingFromBoxes(tmp_boxes,true,hyperParameters));
        axis equal;
        title(num2str(state.Best(end)));
    end
   
end

% Function to minimize
function f = TotalEnergy(x,pool,outImg,hyperParameters)

    % Select the elements
    tmp_boxes =  pool(:,x==1);
    
    % Score the elements
    f = doScoring(tmp_boxes,outImg,hyperParameters);
   
end

% Creates the overlap matrix
function [res_ol, res_minarea] = create_ol_matrix(all)
    
    res_ol = zeros(size(all,2));
    res_minarea = zeros(size(all,2));
    for i=1:size(all,2)
           a = all(:,i);
%            assert(i == a(7));
       for j=1:size(all,2)
           b = all(:,j);
%            assert(j == b(7));
           ol = bb_overlap(a,b);
           res_ol(i,j) = ol;
           
           if ol ==0
               continue;
           end
           minarea = min(bb_area(a) ,bb_area(b));
           res_minarea(i,j) = minarea;
           
       end
    end

end

% Creates the overlap matrix
function [res_ol, res_minarea,A] = create_ol_matrix_new(all)
    
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
           classes = sort([a(5) b(5)]);
           if numel(unique(classes))>1 && ~isequal(classes,[1 4])
               continue;
           end
           
%            if a(5)~=b(5)
%                
%                continue;
%            end
           
           % Calculate the overlap of their BBs
           ol = bb_overlap(a,b);
           res_ol(i,j) = ol;
           
           if ol ==0
               continue;
           end
           constraintRow = zeros(1,size(all,2));
           constraintRow([i j]) = 1;
           A = [A; constraintRow];
           
           % Get the area of the smaller element
           minarea = min(bb_area(a) ,bb_area(b));
           res_minarea(i,j) = minarea;
           
       end
    end

end
