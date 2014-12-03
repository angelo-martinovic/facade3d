function [points,normals,newColors] = BatchLabelPointCloudWithDetections

%     cmvsLocation = '/esat/sadr/amartino/monge3dRight/reconstruction.nvm.cmvs/00/';
    cmvsLocation = '/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/';
    % Read the dense point cloud from CMVS
%     [points,normals,colors]=ReadPointCloudFromPly([cmvsLocation 'models/option-0000.ply']);
    [points,normals,colors]=ReadPointCloudFromPly([cmvsLocation 'models/Paris_RueMonge_part1_ruemonge_107_027_800px.01.ply']);
    % Read camera information from CMVS
    cameras = ImportCameras([cmvsLocation 'cameras_v2.txt']);

    % Mapping between cmvs images and labeled images
    %load('indices.mat');

%     newLabels = zeros(length(points),length(cameras));
    
    unary = zeros(length(points),8);
    % For each camera
    for i=1:length(cameras)
        fprintf('.');
        if mod(i,10)==0
            fprintf('%d\n',i);
        end
        
        % Read the image labeling
        %load(['labelings/monge30Rect_1_' num2str(idx(i)) '_labeling.mat']);
        labelingFilename = [cmvsLocation 'visualizeRotated/output/svm_monge428_newTrain/' cameras{i}.visualizeImageName(1:end-4) '.label_layer2_withDet.mat'];
        if exist(labelingFilename,'file')
            load(labelingFilename);

            %outImg = labeling;
            % The cmvs pictures are resized and (sometimes) rotated
            height = cameras{i}.principalPoint(2)*2;
            width = cameras{i}.principalPoint(1)*2;

            %outImg = imrotate(outImg,90);
            outImg = imresize(outImg,[height width],'nearest');
            detMap = detectionMaps(1).detectionMap;

    %         color=(detMap(:,:,1)~=0.125)+1;


            % Backproject the labeling onto the point cloud
            pointEnergies = BackProjectLabeling(points,detMap,cameras{i});
    %         pointEnergies = ProjectProbabilitiesToPointCloud(points,detMap,cameras{i});
    %         newLabels_i = BackProjectLabeling(points,color,cameras{i});
    %         newLabels_i = newLabels_i-1;
    %         newLabels_i(newLabels_i==-1)=0;
    %         newLabels(:,i) = newLabels_i;
            unary = unary + pointEnergies;
        else
            fprintf('x');
            continue;
        end
        
    end
    unary(unary==0) = 1/8;

    % Each camera projects only on a subset of the point cloud
    % For every point, find the most common label excluding label 0
%     newLabels(newLabels==0)=NaN;
    scores = unary(:,1);

    scores = scores/max(scores);
    
    colors = uint8(scores*255);
    colors = [colors colors colors];
    % Haussmann colormap
%     colormap = [255 0 0;
%                    255  255 0;
%                    128  0   255;
%                    255  128 0;
%                    0    0   255;
%                    128  255 255;
%                    0    255 0;
%                    0    0   255];
%     colormap = colormap / 256;
%        
%     % Mapping colors
%     newColors = colormap(newLabels_agreed,:);
    
    ExportPointCloud('models/Full428_onlyDet_out.ply',points,normals,colors);
   
    % Visualize the point cloud
%     skip=1;
%     fig=VisualizePointCloud(points(1:skip:end,:),newColors(1:skip:end,:),30);
%     view(-90,10);
end