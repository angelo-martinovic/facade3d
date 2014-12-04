    datasetName = 'monge428';
    cmvsLocation = ['/esat/sadr/amartino/' datasetName '/reconstruction.nvm.cmvs/00/'];
    
    filename = '/esat/sadr/amartino/mongeScrape/Paris_RueMonge/part1_ruemonge_107_027_800px_FULL428/Paris_RueMonge_part1_ruemonge_107_027_800px_meshAvImgCol_27x.ply';
    
    
    % Read the mesh from hayko
    if ~exist('points','var')
        [points,~,colors,faces,~]=ReadMeshFromPly(filename);
       
    end
    
    
    
    % Read camera information from CMVS
    cameras = ImportCameras([cmvsLocation 'cameras_v2.txt']);
%     cameras = ImportCamerasNVM([cmvsLocation 'Paris_RueMonge_part1_ruemonge_107_027_800px.nvm']);

    % Mapping between cmvs images and labeled images
    %load('indices.mat');

    histLabelsPerPoint = zeros(length(points),7);
    
    % For each camera
    for i=1:length(cameras)
        fprintf('\b\b\b\b\b\b\b%3i-%3i',i,length(cameras));
        
        
        % Read the image labeling
        %load(['labelings/monge30Rect_1_' num2str(idx(i)) '_labeling.mat']);
%         load([cmvsLocation 'visualizeRect/' cameras{i}.visualizeImageName(1:end-4) '.label_layer2.mat']);
%         outImg = dlmread([cmvsLocation 'visualize/' cameras{i}.visualizeImageName(1:end-4) '.txt']);
        filename = [cmvsLocation 'visualize/' cameras{i}.visualizeImageName(1:end-4) '.txt'];
        if exist(filename,'file')
            gtImg = dlmread(filename);

            %outImg = labeling;
            % The cmvs pictures are resized and (sometimes) rotated
            height = cameras{i}.principalPoint(2)*2;
            width = cameras{i}.principalPoint(1)*2;

            gtImg = imrotate(gtImg,90);
            gtImg = imresize(gtImg,[height width],'nearest');


            % Backproject the labeling onto the point cloud
            newLabels_i = BackProjectLabeling(points,gtImg,cameras{i});
            [indices,~,labels] = find(newLabels_i);
            
            realIndices = sub2ind(size(histLabelsPerPoint),indices,labels);
            histLabelsPerPoint(realIndices) = histLabelsPerPoint(realIndices)+1;
            
        end

    end

    % Each camera projects only on a subset of the point cloud
    % For every point, find the most common label excluding label 0
    fprintf('Majority voting...\n');
    [~,newLabels_agreed] = max(histLabelsPerPoint,[],2);
    newLabels_agreed(sum(histLabelsPerPoint,2)==0) = NaN;
    % Haussmann colormap
    colormap = [255 0 0;
                   255  255 0;
                   128  0   255;
                   255  128 0;
                   0    0   255;
                   128  255 255;
                   0    255 0;
                   0    0   255
                   0    0    0]; % NaN
    colormap = colormap / 256;
       
    % NaN
    newLabels_agreed(isnan(newLabels_agreed))=9;
    
    % Mapping colors
    newColors = colormap(newLabels_agreed,:);
    
    VisualizePointCloud2(points,normals,newColors,10);
%     fprintf('Exporting the point cloud...\n');
    ExportMesh_noNormal(['models/mesh_' datasetName '_GT.ply'],points,round(256*newColors),faces);
   
    groundTruth = newLabels_agreed;
    save('gt/monge428Mesh.mat','groundTruth');

