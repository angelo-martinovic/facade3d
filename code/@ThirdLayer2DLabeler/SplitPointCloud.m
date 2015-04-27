function SplitPointCloud(obj)
    dl = DispatchingLogger.getInstance();
    dl.Log(VerbosityLevel.Info,sprintf('Splitting the scene into individual facades...\n'));
    dl.Log(VerbosityLevel.Info,sprintf('- Loading data...\n'));
    
    %% Load layer 2 unaries
    ss=load(get_adr('pcl_unaries',obj.config,obj.splitName));
    potentials = ss.unary';
    
    %% Load our pcl labeling
    [~,~,colorsNoisy]=ReadPCLFromPly(get_adr('pcl_labeling',obj.config,obj.splitName));
    labeling = Colors2Labels(colorsNoisy,obj.config.cm);

    %% Load splitting data
    ss=load (get_adr('split',obj.config));
    splitLabels = ss.splitLabels;

    %% Extract eval subset
    points = obj.pcl_test.pts';
    colors = obj.pcl_test.rgb';
    labelingGT = obj.pcl_test.lindex';
    
    idxs = obj.pcl_test.p_index';
    
    potentials = potentials(idxs,:);
    splitLabels = splitLabels(idxs,:);
    labeling = labeling(idxs,:);

    %% For each facade
    facadeIDs = unique(splitLabels);
    nFacades = size(facadeIDs,1);

    outputFolder = get_adr('splitOutputDir',obj.config,obj.splitName);
    mkdirIfNotExist(outputFolder);
    
    save(get_adr('facadeIDs',obj.config,obj.splitName),'facadeIDs');
    tic;
    dl.Log(VerbosityLevel.Info,sprintf(' - Performing splitting...\n'));
    for i=1:nFacades
        facadeID = facadeIDs(i);
        if facadeID==0,continue;end % void class
            
        % Get the part of the point cloud corresponding to that facade
        pointSubset = points(splitLabels==facadeID,:);
        vertexSubsetLabeling = labeling(splitLabels==facadeID);
        vertexSubsetLabelingGT = labelingGT(splitLabels==facadeID);
        vertexSubsetPotentials = potentials(splitLabels==facadeID,:); %#ok<NASGU>
        vertexSubsetOrigColors = colors(splitLabels==facadeID,:);

        % Save
        vertexSubsetColors = round(255*obj.config.cm(vertexSubsetLabeling+1,:));
        vertexSubsetColorsGT = round(255*obj.config.cm(vertexSubsetLabelingGT+1,:));

        save(get_adr('splitPotentials',obj.config,obj.splitName,num2str(facadeID)),...
            'vertexSubsetPotentials');
        ExportMesh(get_adr('splitLabeling',obj.config,obj.splitName,num2str(facadeID)),...
            pointSubset,[],vertexSubsetColors,[],[]);
        ExportMesh(get_adr('splitGT',obj.config,obj.splitName,num2str(facadeID)),....
            pointSubset,[],vertexSubsetColorsGT,[],[]);
        ExportMesh(get_adr('splitColors',obj.config,obj.splitName,num2str(facadeID)),...
            pointSubset,[],vertexSubsetOrigColors,[],[]);
    end
    dl.Log(VerbosityLevel.Info,sprintf(' - Done. Elapsed time: %.2f seconds.\n',toc));
end