function g = GetGravityVector(obj,facadeID,facadeNormal)

    
    points = obj.GetAllPoints();
   
    ss=load ([obj.dirName obj.facadeSplit]);
    splitLabels = ss.splitLabels;

    n = size(points,1);
    % Find k nearest neighbors
    k = 4;
    [idx,d]=knnsearch(points,points,'k',k+1);

    % Remove 1st column, corresponding to same element
    idx = idx(:,2:end);
    d = d(:,2:end);

    % Create sparse adjacency matrix
    % i = [11112222...nnnn]
    ii = zeros(1,k*n);
    for i=1:k
        ii(i:k:end)=1:n;
    end

    jj = reshape(idx',1,k*n);
    % ss = reshape(d',1,k*n);

    lab1 = splitLabels(ii);
    lab2 = splitLabels(jj);

    locDiff1 = points(ii(lab1==facadeID & lab2~=facadeID),:);
    locDiff2 = points(jj(lab1==facadeID & lab2~=facadeID),:);

    locSplit = (locDiff1+locDiff2)/2;

    locDiff1 = points(ii(lab1~=facadeID & lab2==facadeID),:);
    locDiff2 = points(jj(lab1~=facadeID & lab2==facadeID),:);
    locSplit = [locSplit; (locDiff1+locDiff2)/2];

    gs = [];
    for rep = 1:5
        % Fit one line
        
        [~,L, inliers] = ransacfitline(locSplit', 0.1, 0);
        proposalG = L(:,1)-L(:,2);
        % y should be positive
        if proposalG(2)<0
            proposalG = -proposalG;
        end
        proposalG = proposalG/norm(proposalG);
        gs = [gs proposalG];
        
        remainingPoints = locSplit(setdiff(1:size(locSplit,1),inliers),:);
        if size(remainingPoints,1)<10
            continue;
        end
        
        % Fit second line
        [~,L, inliers] = ransacfitline(remainingPoints', 0.1, 0);
        proposalG = L(:,1)-L(:,2);
        % y should be positive
        if proposalG(2)<0
            proposalG = -proposalG;
        end
        proposalG = proposalG/norm(proposalG);
        gs = [gs proposalG];

    end

%     g = median(gs,2);
    [~,pos]=min(abs(facadeNormal'*gs)); 
    g = gs(:,pos);
    g = g/norm(g);
end