function SavePotentialsAndLabels(obj)
    dl = DispatchingLogger.getInstance();
    cf = DatasetConfig.getInstance();
    
    outputPCLName = obj.getOutputName();
    
    % Point cloud
    nPointsFull = size(obj.scene.pts',1);
    
    % Indices of the labeled subset
    idxs = find(obj.scene.flag==2)';
    
    % Save unaries
    unary = zeros(cf.nClasses,nPointsFull);
    unary(:,idxs) = obj.unary; %#ok<NASGU>
    
    save(get_adr('pcl_3DCRF_unaries',outputPCLName),'unary');
    
    % Save and evaluate labeled point cloud
    dl.Log(VerbosityLevel.Info,sprintf(' - Saving and evaluating the point clouds...\n'));
    
    % MAP (no pairwise terms)
    dl.Log(VerbosityLevel.Info,sprintf(' - MAP...\n'));
    colors =  zeros(nPointsFull,3);
    colors(idxs,:) = round(255*cf.cm(obj.labelsMAP+1,:));
    outputFilename = get_adr('pcl_3DMAP_labeling',outputPCLName);
    ExportMesh(outputFilename,obj.scene.pts',[],colors,[],[]);
    
    score = EvaluatePointCloudLabeling(outputFilename); %#ok<NASGU>
%     disp(score);
    scoreFilename = get_adr('score',outputFilename);
    save(scoreFilename,'score');
    
    % CRF
    dl.Log(VerbosityLevel.Info,sprintf(' - CRF...\n'));
    colors =  zeros(nPointsFull,3);
    colors(idxs,:) = round(255*cf.cm(obj.labelsCRF+1,:));
    outputFilename = get_adr('pcl_3DCRF_labeling',outputPCLName);
    ExportMesh(outputFilename,obj.scene.pts',[],colors,[],[]);
    
    score = EvaluatePointCloudLabeling(outputFilename); %#ok<NASGU>
%     disp(score);
    scoreFilename = get_adr('score',outputFilename);
    save(scoreFilename,'score');
    
    dl.Log(VerbosityLevel.Info,sprintf('Done.\n'));

end