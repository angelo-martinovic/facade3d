% Read the point cloud and camera info
dirName = '/esat/sadr/amartino/monge428New/data/';

[points,normals,~]=ReadPCLFromPly([dirName 'pmvs_pcloud_colors_normals.ply']);
[pointsPCLtest,~,colorsPCLtest]=ReadPCLFromPly([dirName 'pmvs_pcloud_test.ply']);
[pointsPCLtrain,~,colorsPCLtrain]=ReadPCLFromPly([dirName 'pmvs_pcloud_train.ply']);
assert(isequal(pointsPCLtest,pointsPCLtrain));

labelsTest = Colors2Labels(colorsPCLtest);
labelsTrain = Colors2Labels(colorsPCLtrain);

pointsPCL = pointsPCLtest(labelsTest~=0 | labelsTrain~=0,:);

idxs = knnsearch(points,pointsPCL);
points = points(idxs,:);
normals = normals(idxs,:);

cameras = ImportCameras([dirName 'cameras.txt']);

depths = NaN(length(points),length(cameras));

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
    p_fPlane_avg = median(pointSubset,1)';

    % 2. Fit a plane
    [p_fPlane,n_fPlane] = FitPlane(pointSubset);

    % Invert the RANSAC plane normal if it has the wrong direction
    if dot(n_fPlane,n_fPlane_avg)<0
        n_fPlane = - n_fPlane;
    end

    % Find the distance between every 3d point and the facade plane
    q = pointSubset';
    c = num2cell(q,1);
    distances = cellfun(@(x) dot(x-p_fPlane,n_fPlane) , c );

    % Projecting the depth information to the camera
%         depth = ProjectDepthToCamera(pointSubset,distances,cameras{i});
    depths(newLabels_i>0,i) = distances;
%         figure(1);imagesc(depth);colorbar;
%         depthFileName = [depthDir  cameras{i}.originalImageFilename(1:end-4) '.mat'];
%         save(depthFileName,'depth');
pb.progress;
end
pb.stop;

depthDoUsPart = nanmean(depths,2);  % heh.
depth = NaN(length(pointsPCLtest),1);
depth(labelsTest~=0 | labelsTrain~=0) = depthDoUsPart;
save([dirName 'pmvs_depth.mat'],'depth');