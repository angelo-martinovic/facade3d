clear;
datasetName = 'monge428';

addpath ransac/

cmvsLocation = ['/esat/sadr/amartino/' datasetName '/reconstruction.nvm.cmvs/00/'];
% Read the dense point cloud from CMVS
[points,normals,colors]=ReadPointCloudFromPly([cmvsLocation 'models/Paris_RueMonge_part1_ruemonge_107_027_800px.01.ply']);

% Read camera information from CMVS
cameras = ImportCameras([cmvsLocation 'cameras_v2.txt']);

    %% If needed, this calculates the ground plane
    camerapos = zeros(length(cameras),3);
    % Calculate 3d positions of cameras
    for i=1:length(cameras)
        bla=cameras{i}.P;
        Phat=bla(:,1:3); 
        Fhat=bla(:,4); 
        camerapos(i,:)=Phat\Fhat;
    end

    camerapos = -camerapos;

    fig=VisualizeCameras(camerapos,2);

    % Find ground plane
     [p_gPlane,n_gPlane] = FitPlane(camerapos);

    %disp(p_gPlane);
    %disp(n_gPlane);

    points = points(1:10:end,:);
    normals = normals(1:10:end,:);
    colors = colors(1:10:end,:);
    
    points = [points; camerapos];
    normals = [normals; ones(size(camerapos))];
    colors = [colors; ones(size(camerapos))];
    
    
    VisualizePointCloud2(points,normals,colors,1);
    
    %% For each camera, find the depth map
    
    
    newLabels = zeros(length(points),length(cameras),'int8');
%     depths = zeros(length(points),length(cameras));
    for i=1:length(cameras)
        fprintf('\b\b\b\b\b\b\b%3i-%3i',i,length(cameras));

        % First, find the subset of points visible from the camera by
        % projecting a 'dummy' labeling
        outImg = ones(floor(2*cameras{i}.principalPoint(2)),floor(2*cameras{i}.principalPoint(1)));
        newLabels_i = BackProjectLabeling(points,outImg,cameras{i});
        newLabels(:,i) = newLabels_i;
        
        % Find the labeled points
        pointSubset = points(newLabels(:,i)>0,:);
        normalSubset = normals(newLabels(:,i)>0,:);

        visiblePointsIndices = find(newLabels(:,i)>0);
        
        % Fit a facade plane
        
        % 1. The average of all point normals
        n_fPlane_avg = mean(normalSubset,1)';
        p_fPlane_avg = median(pointSubset,1)';
        
        % 2. Fit a plane with RANSAC
        [p_fPlane,n_fPlane] = FitPlane(pointSubset(1:10:end,:));
        
       % p_fPlane=p_fPlane_avg;
        %n_fPlane = n_fPlane_avg;
        % Invert the RANSAC plane normal if it has the wrong direction
        if dot(n_fPlane,n_fPlane_avg)<0
            n_fPlane = - n_fPlane;
        end
         
        % Make sure that the facade plane is orthogonal to the ground plane
%         if dot(n_gPlane,n_fPlane)>0.1
%             warning('Possible miscalculation of the facade plane.');
%         end
        
        % Find the distance between every 3d point and the facade plane
        q = pointSubset';
        c = num2cell(q,1);
        distances = cellfun(@(x) dot(x-p_fPlane,n_fPlane) , c );

%         depths(visiblePointsIndices,i) = distances;
        % Projecting the depth information to the camera
        img = ProjectDepthToCamera(pointSubset,distances,cameras{i});
        figure(1);clf;
        imagesc(img);
        %imagesc(imrotate(img,-90));
%         colorbar;
%         figure;imagesc(img>0)
%         figure;imagesc(img<0)
        
%         visFilename = cameras{i}.visualizeImageName;
%         depthFileName = [cmvsReconstructionPath 'visualize/' visFilename(1:end-4) '_depth.txt'];
%         dlmwrite(depthFileName,img,' ');
    end
