function SavePotentialsAndLabels(obj)
    dl = DispatchingLogger.getInstance();
    
    outputPCLName = obj.getOutputName();
    
    % Save unaries
    unary = zeros(obj.config.nClasses,size(obj.pointsFull,1));
    unary(:,obj.idxs) = obj.unary; %#ok<NASGU>
    
    save(get_adr('pcl_3DCRF_unaries',obj.config,outputPCLName),'unary');

    gtFilename = get_adr('pcl_gt_test',obj.config);
    
    % Save and evaluate labeled point cloud
    dl.Log(VerbosityLevel.Info,sprintf(' - Saving and evaluating the point clouds...\n'));
    
    % MAP (no pairwise terms)
    dl.Log(VerbosityLevel.Info,sprintf(' - MAP...\n'));
    colors =  zeros(size(obj.pointsFull,1),3);
    colors(obj.idxs,:) = round(255*obj.config.cm(obj.labelsMAP+1,:));
    outputFilename = get_adr('pcl_3DMAP_labeling',obj.config,outputPCLName);
    ExportMesh(outputFilename,obj.pointsFull,[],colors,[],[]);
    
    score = EvaluateMeshLabeling(outputFilename,gtFilename);
    disp(score);
    scoreFilename = get_adr('score',obj.config,outputFilename);
    save(scoreFilename,'score');
    
    % CRF
    dl.Log(VerbosityLevel.Info,sprintf(' - CRF...\n'));
    colors =  zeros(size(obj.pointsFull,1),3);
    colors(obj.idxs,:) = round(255*obj.config.cm(obj.labelsCRF+1,:));
    outputFilename = get_adr('pcl_3DCRF_labeling',obj.config,outputPCLName);
    ExportMesh(outputFilename,obj.pointsFull,[],colors,[],[]);
    
    score = EvaluateMeshLabeling(outputFilename,gtFilename);
    disp(score);
    scoreFilename = get_adr('score',obj.config,outputFilename);
    save(scoreFilename,'score');
    
    dl.Log(VerbosityLevel.Info,sprintf('Done.\n'));

end