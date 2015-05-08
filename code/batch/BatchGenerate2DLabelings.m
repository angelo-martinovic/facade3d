% Inputs:  images to project to, and the name of the labeled point cloud
%  e.g. 'monge428_3D_3DCRF.ply'
function outputDir = BatchGenerate2DLabelings(imageNames,modelName)
    
    dl = DispatchingLogger.getInstance();
    cf = DatasetConfig.getInstance();
    
    % Read the point cloud info
    [pointsPCLtest,~,colorsPCLtest]=ReadPCLFromPly(get_adr('pcl_labeling',modelName));
    labelsTest = Colors2Labels(colorsPCLtest,cf.cm);

    % Remove non-labeled points
    pointsSubset = pointsPCLtest(labelsTest~=0,:);
    labelsSubset = labelsTest(labelsTest~=0);

    % Read the camera info
    cameras = ImportCameras(get_adr('cameras'));
    
    % Output folder
    outputDir = get_adr('projectedTo2D',modelName);
    mkdirIfNotExist(outputDir);

    tic;
    dl.Log(VerbosityLevel.Info,...
        sprintf(' - Projecting the %s point cloud labeling to images...\n',modelName));
    pb = ProgressBar(length(cameras));
    for i=1:length(cameras)
        % Check if the camera appears in the list of images in the test set
        if isempty( find(strcmp(imageNames,cameras{i}.originalImageFilename(1:end-4)),1) )
            pb.progress;
            continue;
        end

        % Back-projecting the labeling information through the camera
        labeledImage = ProjectToCamera2(pointsSubset,labelsSubset,cameras{i},cf.cm);
        labeledImageFileName = [outputDir cameras{i}.originalImageFilename(1:end-4) '.png'];
        imwrite(labeledImage,labeledImageFileName);
        
        pb.progress;
    end
    pb.stop;
    dl.Log(VerbosityLevel.Info,sprintf(' - Elapsed %d seconds.\n',toc));
end