function score = scoreWindowGrid(boxes_in, hyperParameters)

    all_align_score = 0;
    
    if size(boxes_in,2)==0
        warning('scoreWindowGrid - No objects found!');
        score = 0 ;
        return;
    end
    
    % Ignore door class for grid alignment
    scoreClasses = setdiff(hyperParameters.objClasses,hyperParameters.doorClass);
    
    % For each object class separately
    for label=scoreClasses

        % Get bounding boxes of the object class
        boxes = boxes_in(:,boxes_in(5,:) == label);

        if size(boxes,2)<=1
            continue;
        end

        % Count how many objects' borders are within the 'tukey window'
        % i.e. will be aligned after optimization
        if label==1
            ddw =hyperParameters.win_ddw;
            ddh =hyperParameters.win_ddh;
        else
            ddw =hyperParameters.balc_ddw;
            ddh =hyperParameters.balc_ddh;
        end
        % Horizontally
        aw = [pdist(boxes(1,:)') pdist(boxes(3,:)')]<ddw;
        
        % Vertically
        ah = [pdist(boxes(2,:)') pdist(boxes(4,:)')]<ddh;
        
        % Score is negative, as we are minimizing
        if hyperParameters.legacy
            align_score = -2 * sum([aw ah]); 
            all_align_score = all_align_score + align_score/size(boxes,2); 
        else
            align_score = -sum([aw ah]) / numel([aw ah]);
            all_align_score = all_align_score + align_score; % Normalized
        end
          
    end

    if hyperParameters.legacy
        score = all_align_score; 
    else
      score = all_align_score/numel(scoreClasses);
    end
end
