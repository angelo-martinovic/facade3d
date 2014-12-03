
% BatchGenerate2DLabelings('monge428New_fullRes_3D.ply')
function BatchGenerate2DLabelings(modelName)
    % Read the point cloud and camera info
    dirName = '/esat/sadr/amartino/monge428New/data/';

    % modelName = 'monge428New_fullRes_3D.ply';
%     [points,~,~]=ReadPCLFromPly([dirName 'mesh_colors_normals.ply']);
    [pointsPCLtest,~,colorsPCLtest]=ReadPCLFromPly([dirName 'work/pcl/models/' modelName]);

%     assert(isequal(size(pointsPCLtest),size(points)));

    labelsTest = Colors2Labels(colorsPCLtest);

    pointsSubset = pointsPCLtest(labelsTest~=0,:);
    labelsSubset = labelsTest(labelsTest~=0);

    % idxs = knnsearch(points,pointsPCL);
    % points = points(idxs,:);

    cameras = ImportCameras([dirName 'cameras.txt']);

    file_str_idx = LoadFilenames(dirName,'eval');
    nImages = length(file_str_idx);
    
    outputDir = [dirName 'work/output-' modelName '/'];
    if ~exist(outputDir,'dir')
        mkdir(outputDir);
    end

    tic;
    pb = ProgressBar(length(cameras));
    for i=1:length(cameras)
        if isempty( find(strcmp(file_str_idx,cameras{i}.originalImageFilename(1:end-4)),1) )
            pb.progress;
            continue;
        end
        % First, find the subset of points visible from the camera by
        % projecting a 'dummy' labeling
    %     outImg = ones(floor(2*cameras{i}.principalPoint(2)),floor(2*cameras{i}.principalPoint(1)));
    %     newLabels_i = BackProjectLabeling(points,outImg,cameras{i});

        % Find the labeled points
    %     pointSubset = points(newLabels_i>0,:);
    %     colorSubset = colorsPCLtest(newLabels_i>0,:);


        % Projecting the labeling information to the camera
        labeledImage = ProjectToCamera2(pointsSubset,labelsSubset,cameras{i});
    %     figure(1);imagesc(labeledImage);colorbar;
        labeledImageFileName = [outputDir cameras{i}.originalImageFilename(1:end-4) '.png'];
        imwrite(labeledImage,labeledImageFileName);
    pb.progress;
    end
    pb.stop;
    projectionTo2DTime = toc;
    fprintf('Elapsed %d seconds.\n',projectionTo2DTime);
end