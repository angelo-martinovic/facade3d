function SplitPointCloud(obj)
    %% Load layer 2 unaries
    load(get_adr('3D_L2_unaries',obj.config,obj.splitName));
    potentials = unary';

    %% Load original pcl
    [points,~,colors] = ReadPCLFromPly(get_adr('pcl',obj.config));

    %% Load our pcl labeling
    [pointsNoisy,~,colorsNoisy]=ReadPCLFromPly(get_adr('3D_L2_labeling',obj.config,obj.splitName));
    assert(all(all(pointsNoisy-points<1e-3)))
    labeling = Colors2Labels(colorsNoisy,obj.config.cm);

    %% Load test GT
    [pointsGT,~,colorsGT]=ReadPCLFromPly(get_adr('pcl_gt_test',obj.config));
    assert(all(all(pointsGT-points<1e-3)))
    labelingGT = Colors2Labels(colorsGT,obj.config.cm);

    %% Load splitting data
    ss=load (get_adr('split',obj.config));
    splitLabels = ss.splitLabels;

    %% Get eval subset
    idxs = find(labeling~=0);

    points = points(idxs,:);
    potentials = potentials(idxs,:);
    colors= colors(idxs,:);
    splitLabels = splitLabels(idxs,:);
    labelingGT = labelingGT(idxs,:);
    labeling = labeling(idxs,:);

    %% For each facade
    facadeIDs = unique(splitLabels);
    nFacades = size(facadeIDs,1);

    outputFolder = get_adr('splitOutputDir',obj.config,obj.splitName);
    mkdirIfNotExist(outputFolder);
    
    save(get_adr('facadeIDs',obj.config,obj.splitName),'facadeIDs');
    tic;
    for i=1:nFacades

        facadeID = facadeIDs(i);

        if facadeID==0 % void class
            continue;
        end

        % Get the part of the point cloud corresponding to that facade
        pointSubset = points(splitLabels==facadeID,:);
        vertexSubsetLabeling = labeling(splitLabels==facadeID);
        vertexSubsetLabelingGT = labelingGT(splitLabels==facadeID);
        vertexSubsetPotentials = potentials(splitLabels==facadeID,:);
        vertexSubsetOrigColors = colors(splitLabels==facadeID,:);

        % Save
        vertexSubsetColors = round(255*obj.config.cm(vertexSubsetLabeling+1,:));
        vertexSubsetColorsGT = round(255*obj.config.cm(vertexSubsetLabelingGT+1,:));

        save(get_adr('splitPotentials',obj.config,obj.splitName,num2str(facadeID)),'vertexSubsetPotentials');
        ExportMesh(get_adr('splitLabeling',obj.config,obj.splitName,num2str(facadeID)),pointSubset,[],vertexSubsetColors,[],[]);
        ExportMesh(get_adr('splitGT',obj.config,obj.splitName,num2str(facadeID)),      pointSubset,[],vertexSubsetColorsGT,[],[]);
        ExportMesh(get_adr('splitColors',obj.config,obj.splitName,num2str(facadeID)),  pointSubset,[],vertexSubsetOrigColors,[],[]);
    end
    facadeSplitTime = toc;
    fprintf('Elapsed %d seconds.\n',facadeSplitTime);
end