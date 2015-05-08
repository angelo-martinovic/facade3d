function Run3DCRF(obj)
    
    dl = DispatchingLogger.getInstance();
    cf = DatasetConfig.getInstance();
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Using the following energy function:\n'));
    
    % Indices of the labeled subset
    idxs = find(obj.scene.flag==2)';
    
    points = obj.scene.pts(:,idxs)';
    unaries = obj.unaries;
    pairwiseCost = cf.c3D.crf.weightPairwise;
    
    unary = 0;
    enFunString = ' - ';
    for i=1:length(unaries)
        if unaries(i).weight>0
            enFunString = [enFunString sprintf('%.2f*%s + ',unaries(i).weight,unaries(i).name')]; %#ok<AGROW>
            
            unary = unary + unaries(i).weight * unaries(i).unary;
        end
    end
    
    enFunString = [enFunString sprintf(sprintf('%.2f*pairwise\n',pairwiseCost))];
    dl.Log(VerbosityLevel.Info,enFunString);

    if sum(unary(:))==0
        dl.Log(VerbosityLevel.Error,sprintf('No unaries found!'));
        fatal();
    end
    
    n = size(points,1);
    [~,obj.labelsMAP] = min(unary);

    % Graph cuts
    %% Pairwise potentials
    % Find k nearest neighbors and the distances between them
    k = 4;
    [idx,d]=knnsearch(points,points,'k',k+1); %#ok<ASGLU>

    % Remove 1st column, corresponding to same element
    idx = idx(:,2:end);
%     d = d(:,2:end); % distance-based weighting

    % Create sparse adjacency matrix
    % i = [11112222...nnnn]
    ii = zeros(1,k*n);
    for i=1:k
        ii(i:k:end)=1:n;
    end

    jj = reshape(idx',1,k*n);
    
    % Pairwise costs: w = exp(-(d/sigma)^2)
%     ss = reshape(d',1,k*n);   % distance-based weighting
%     sigma = mean(ss(:));      % distance-based weighting
%     ss = 3* exp( - (ss/sigma).^2  );  % distance-based weighting  % When ss==mean(ss), penalty should be 1
    
    ss=1;
    pairwise = sparse(ii,jj,ss,n,n);

    %% Label cost
    C = size(unary,1);
    labelcost = ones(C,C);
    labelcost(1:C+1:end)=0;

    lambda = pairwiseCost;
    pairwise = lambda * pairwise;

    segclass = obj.labelsMAP-1;

    dl.Log(VerbosityLevel.Info,sprintf(' - Running the graph cut...\n'));
    tic;
    [obj.labelsCRF,~,~] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    dl.Log(VerbosityLevel.Info,sprintf(' - Done. Elapsed time: %.2f seconds.\n',toc));
    obj.labelsCRF = obj.labelsCRF + 1;

    obj.unary = unary;
    
end