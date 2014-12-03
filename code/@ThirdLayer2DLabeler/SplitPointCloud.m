function SplitPointCloud(obj)
    %% Load points
    load([obj.dirName 'work/pcl/probs/' obj.splitName '.mat']);
    potentials = unary';

    %% Load original pcl
    [points,colors] = obj.GetAllPoints();

    %% Load our pcl labeling
    [pointsNoisy,~,colorsNoisy]=ReadPCLFromPly([obj.dirName 'work/pcl/models/' obj.splitName '.ply']);
    assert(all(all(pointsNoisy-points<1e-3)))
    labeling = Colors2Labels(colorsNoisy,obj.cm);

    %% Load test GT
    [pointsGT,~,colorsGT]=ReadPCLFromPly([obj.dirName obj.gtTestFilename]);
    assert(all(all(pointsGT-points<1e-3)))
    labelingGT = Colors2Labels(colorsGT,obj.cm);

    %% Load splitting data
    ss=load ([obj.dirName obj.facadeSplit]);
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

    outputFolder = [obj.dirName 'work/pcl/split/' obj.splitName '/'];
    if ~exist(outputFolder,'dir')
        mkdir(outputFolder);
    end
    save([outputFolder obj.modelName '_facadeIDs.mat'],'facadeIDs');
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
        cmap = HaussmannColormap();
        vertexSubsetColors = cmap(vertexSubsetLabeling+1,:);
        vertexSubsetColorsGT = cmap(vertexSubsetLabelingGT+1,:);

        save([outputFolder obj.modelName '_split_' num2str(facadeID) '_potentials.mat'],'vertexSubsetPotentials');
        ExportMesh([outputFolder obj.modelName '_split_' num2str(facadeID) '_labeling.ply'],pointSubset,[],vertexSubsetColors,[],[]);
        ExportMesh([outputFolder obj.modelName '_split_' num2str(facadeID) '_GT.ply'],      pointSubset,[],vertexSubsetColorsGT,[],[]);
        ExportMesh([outputFolder obj.modelName '_split_' num2str(facadeID) '_colors.ply'],  pointSubset,[],vertexSubsetOrigColors,[],[]);
    end
    facadeSplitTime = toc;
    fprintf('Elapsed %d seconds.\n',facadeSplitTime);
end