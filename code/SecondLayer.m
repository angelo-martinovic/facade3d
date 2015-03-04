function outputPCLName = SecondLayer(datasetConfig)

    dl = DispatchingLogger.getInstance();

    dl.Log(VerbosityLevel.Info,sprintf('Running point cloud labeling with a 3D CRF ...\n'));
    
    % Load point cloud
    [pointsFull,~,colorsFull] = ReadPCLFromPly(get_adr('pcl_gt_test',datasetConfig));

    % Indices of the labeled subset
    idxs = find(sum(colorsFull,2)~=0);
    points = pointsFull(idxs,:);

    dl.Log(VerbosityLevel.Info,sprintf(' - Found the following unary potentials: \n'));
    probs = [];
    weights = [];
    unaryNames = {};
  
    % Load 3D labeling
    filename = get_adr('pcl_unaries',datasetConfig,'3D');
    if exist(filename,'file')
        s_L = load(filename);
        assert(size(s_L.prb,1)==datasetConfig.nClasses,'Wrong file format!'); % TODO
        assert(size(s_L.prb,2)==size(idxs,1),'Wrong number of points!'); % TODO
        probs_3D = s_L.prb(1:datasetConfig.nClasses,:); 
        probs = cat(3,probs,probs_3D);
        unaryNames = [unaryNames {'3D'}];
        weights = [weights datasetConfig.c3D.crf.weight3DClassification];
        dl.Log(VerbosityLevel.Info,sprintf(' - - 3D classification\n'));
    else
        if datasetConfig.c3D.crf.weight3DClassification~=0
            dl.Log(VerbosityLevel.Error,sprintf(' - No 3D potentials found!\n'));
            error('Critical error. Terminating.');
        end
    end


    % Load 2D labeling
    filename = get_adr('pcl_unaries',datasetConfig,'2D');
    if exist(filename,'file')
        s_L = load(filename);
        probs_2D_seg = s_L.unary(3+1:3+datasetConfig.nClasses,idxs);
        probs = cat(3,probs,probs_2D_seg);
        unaryNames = [unaryNames {'2D'}];
        weights = [weights datasetConfig.c3D.crf.weight2DClassification];
        dl.Log(VerbosityLevel.Info,sprintf(' - - 2D classification\n'));

        nDet = length(s_L.unaryDet);
        detNames = cell(1,nDet);
        for i=1:nDet
            detNames{i} = s_L.unaryDet{i}.name;
            probs = cat(3,probs,s_L.unaryDet{i}.unary(3+1:3+datasetConfig.nClasses,idxs));
            dl.Log(VerbosityLevel.Info,sprintf(' - - 2D detector: %s\n',detNames{i}));
            
        end
        unaryNames = [unaryNames detNames];
        weights = [weights datasetConfig.c3D.crf.weights2DDetectors];
        
        if nDet~=length(datasetConfig.c3D.crf.weights2DDetectors)
            dl.Log(VerbosityLevel.Error,...
                sprintf(' - Mismatch between expected (%d) and received (%d) number of detectors!\n',...
                        length(datasetConfig.c3D.crf.weights2DDetectors),nDet));
            error('Critical error. Terminating.');
        end

    else
        if datasetConfig.c3D.crf.weight2DClassification~=0 
            dl.Log(VerbosityLevel.Error,sprintf(' - No 2D classification potentials found!\n'));
            error('Critical error. Terminating.');
        end
    end

    % Setup unary potentials
    nUnary = length(unaryNames);
    unaries = struct('unary',[],'name',[],'weight',[]);
    for i=1:nUnary
        p = probs(:,:,i);
        p(p==0) = eps;
        p = p-min(p(:));
        p = p/max(p(:));
        unaries(i).unary = -log(bsxfun(@rdivide,p,sum(p)));
        unaries(i).name = unaryNames{i};
        unaries(i).weight = weights(i);
    end

    % Run the CRF
    [~, labelsCRF, outputPCLName, unarySub] = ...
        LabelPointCloudWithUnaries(points,unaries,datasetConfig.c3D.crf.weightPairwise);
    
    % Save unaries
    unary = zeros(datasetConfig.nClasses,size(pointsFull,1));
    unary(:,idxs) = unarySub; %#ok<NASGU>
    save(get_adr('pcl_3DCRF_unaries',datasetConfig,outputPCLName),'unary');

    % Save labeled point cloud
    dl.Log(VerbosityLevel.Info,sprintf(' - Saving the point cloud...\n'));
    colors =  zeros(size(pointsFull,1),3);
    colors(idxs,:) = round(255*datasetConfig.cm(labelsCRF+1,:));
    ExportMesh(get_adr('pcl_3DCRF_labeling',datasetConfig,outputPCLName),pointsFull,[],colors,[],[]);
    dl.Log(VerbosityLevel.Info,sprintf('Done.\n'));
    
%     outputPCLName = [outputPCLName '_3DCRF'];

end

%%
function [labelsMAP, labelsCRF, outputPCLName, unary] = LabelPointCloudWithUnaries(points,unaries,pairwiseCost)
    
    dl = DispatchingLogger.getInstance();
    dl.Log(VerbosityLevel.Info,sprintf(' - Using the following energy function:\n'));
    
    outputPCLName = '';
    unary = 0;
    connector = '';
    enFunString = ' - ';
    for i=1:length(unaries)
        if unaries(i).weight>0
            outputPCLName = [outputPCLName connector unaries(i).name];    %#ok<AGROW>
            enFunString = [enFunString sprintf('%.2f*%s + ',unaries(i).weight,unaries(i).name')]; %#ok<AGROW>
            
            if strcmp(connector,'')
                connector='+';
            end
            unary = unary + unaries(i).weight * unaries(i).unary;
        end
    end
    enFunString = [enFunString sprintf(sprintf('%.2f*pairwise\n',pairwiseCost))];
    dl.Log(VerbosityLevel.Info,enFunString);

    if sum(unary(:))==0
        dl.Log(VerbosityLevel.Error,sprintf('No unaries found!'));
        error('Critical error. Terminating.');
    end
    
    n = size(points,1);
    [~,labelsMAP] = min(unary);

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

    segclass = labelsMAP-1;

    dl.Log(VerbosityLevel.Info,sprintf(' - Running the graph cut...\n'));
    tic;
    [labelsCRF,~,~] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    CRFtime = toc;
    dl.Log(VerbosityLevel.Info,sprintf(' - Done. Elapsed time: %.2f seconds.\n',CRFtime));
    labelsCRF = labelsCRF + 1;

end

