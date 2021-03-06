function calculateDepthMaps()
    datasetConfig = DatasetConfig.getInstance();
    dl = DispatchingLogger.getInstance();
    
    % Read the point cloud and camera info
    dirName = datasetConfig.dataLocation;
    
    depthFilename = [dirName,datasetConfig.depth];
    
    if exist(depthFilename,'file')
        t= dir(depthFilename);
        dl.Log(VerbosityLevel.Debug,...
            sprintf(' - - Found depth info in %s, created on %s ...\n',depthFilename,t.date));
        return;
    end
   
    [points,normals,~]=ReadPCLFromPly([dirName,datasetConfig.pointCloud]);
    [pointsPCLtest,~,colorsPCLtest]=ReadPCLFromPly([dirName,datasetConfig.groundTruthTest]);
    [pointsPCLtrain,~,colorsPCLtrain]=ReadPCLFromPly([dirName,datasetConfig.groundTruthTrain]);
    assert(isequal(pointsPCLtest,pointsPCLtrain));

    labelsTest = Colors2Labels(colorsPCLtest,datasetConfig.cm);
    labelsTrain = Colors2Labels(colorsPCLtrain,datasetConfig.cm);

    pointsPCL = pointsPCLtest(labelsTest~=0 | labelsTrain~=0,:);

    idxs = knnsearch(points,pointsPCL);
    points = points(idxs,:);
    normals = normals(idxs,:);

    cameras = ImportCameras([dirName,datasetConfig.cameras]);

%     depths = NaN(length(points),length(cameras));
    
    depthsAcc = zeros(length(points),1);
    depthsCnt = zeros(length(points),1);

    pb = ProgressBar(length(cameras));
    for i=1:length(cameras)
        % First, find the subset of points visible from the camera by
        % projecting a 'dummy' labeling
        outImg = ones(floor(2*cameras{i}.principalPoint(2)),floor(2*cameras{i}.principalPoint(1)));
        newLabels_i = BackProjectLabeling(points,outImg,cameras{i});

        % Find the labeled points
        pointSubset = points(newLabels_i>0,:);
        normalSubset = normals(newLabels_i>0,:);

        % Fit a facade plane
        % 1. The average of all point normals
        n_fPlane_avg = mean(normalSubset,1)';
%         p_fPlane_avg = median(pointSubset,1)';

        % 2. Fit a plane
        fPlane = FitPlane(pointSubset);

        % Invert the RANSAC plane normal if it has the wrong direction
        if dot(fPlane.n,n_fPlane_avg)<0
            fPlane.n = - fPlane.n;
        end

        % Find the distance between every 3d point and the facade plane
        q = pointSubset';
        c = num2cell(q,1);
        distances = cellfun(@(x) dot(x-fPlane.p,fPlane.n) , c );

        % Projecting the depth information to the camera
    %         depth = ProjectDepthToCamera(pointSubset,distances,cameras{i});
%         depths(newLabels_i>0,i) = distances;
        depthsAcc(newLabels_i>0) = depthsAcc(newLabels_i>0)+distances';
        depthsCnt(newLabels_i>0) = depthsCnt(newLabels_i>0)+1;
    %         figure(1);imagesc(depth);colorbar;
    %         depthFileName = [depthDir  cameras{i}.originalImageFilename(1:end-4) '.mat'];
    %         save(depthFileName,'depth');
    pb.progress;
    end
    pb.stop;

%     save('tmp.mat','depths');
    depthDoUsPart = depthsAcc./depthsCnt;
    
%     depthDoUsPart = nanmean(depths,2);  % heh.
    
    depth = NaN(length(pointsPCLtest),1);
    depth(labelsTest~=0 | labelsTrain~=0) = depthDoUsPart; %#ok<NASGU>
    save(depthFilename,'depth');
end