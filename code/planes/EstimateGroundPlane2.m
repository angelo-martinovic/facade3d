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

%     fig=VisualizeCameras(camerapos,2);

    % Find ground plane
     [p_gPlane,n_gPlane] = FitPlane(camerapos);

    %disp(p_gPlane);
    %disp(n_gPlane);

    points = points(1:100:end,:);
    normals = normals(1:100:end,:);
    colors = colors(1:100:end,:);
    
    points = [points; camerapos];
    normals = [normals; ones(size(camerapos))];
    colors = [colors; ones(size(camerapos))];
    
    % Create the 4 support points for the ground plane
    pos1 = -100;
    pos2 = 100;
    faceX = [pos1 pos2 pos2 pos1]';
    faceZ = [pos1 pos1 pos2 pos2]';
    A = n_gPlane(1);
    B = n_gPlane(2);
    C = n_gPlane(3);
    x1 = p_gPlane(1);
    y1 = p_gPlane(2);
    z1 = p_gPlane(3);
    faceY = y1 - A/B*(faceX-x1) - C/B*(faceZ-z1);
    
    % Add the points
    points = [points; [faceX faceY faceZ]];
    normals = [normals; ones(4,3)];
    colors = [colors; ones(4,3)];
    
    % Face connecting the last 4 points
    faces = [4 length(points)-1 length(points)-2 length(points)-3 length(points)-4];
    
%     VisualizePointCloud2(points,normals,colors,1);
    VisualizePointCloud(points,normals,colors,faces);

