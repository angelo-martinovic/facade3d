function [points,normals,newColors] = BatchLabelPointCloudObjectIDs

    cmvsLocation = '/esat/sadr/amartino/monge3dRight/reconstruction.nvm.cmvs/00/';
    % Read the dense point cloud from CMVS
    [points,normals,colors]=ReadPointCloudFromPly([cmvsLocation 'models/option-0000.ply']);
    
    % Read camera information from CMVS
    cameras = ImportCameras([cmvsLocation 'cameras_v2.txt']);

    % Mapping between cmvs images and labeled images
    %load('indices.mat');

    newLabels = zeros(length(points),length(cameras));
    
    targetLabel = 1;
    maxObjectID = 0;
    % For each camera
    for i=1:length(cameras)
        fprintf('.');
        
        
        % Read the image labeling
        %load(['labelings/monge30Rect_1_' num2str(idx(i)) '_labeling.mat']);
%         load([cmvsLocation 'visualizeRect/' cameras{i}.visualizeImageName(1:end-4) '.label_layer2.mat']);
        outImg = dlmread([cmvsLocation 'visualize/' cameras{i}.visualizeImageName(1:end-4) '.txt']);

        %outImg = labeling;
        % The cmvs pictures are resized and (sometimes) rotated
        height = cameras{i}.principalPoint(2)*2;
        width = cameras{i}.principalPoint(1)*2;
    
        %outImg = imrotate(outImg,90);
        outImg = imresize(outImg,[height width],'nearest');
        
        outImg = bwlabel(outImg==targetLabel);
        
        % Assign a unique ID to each target object over all images
        relabeled = outImg;
        relabeled = relabeled + maxObjectID;
        relabeled(outImg==0)=0;
        
        % Counter for unique objects per image
        maxObjectID = maxObjectID+max(outImg(:));
    
        outImg = relabeled;
        
        % Backproject the labeling onto the point cloud
        newLabels_i = BackProjectObjectIDs(points,outImg,cameras{i});
        newLabels(:,i) = newLabels_i;

    end

    % Rows with sum 0 are not labeled, skip them
    newLabelSum = sum(newLabels,2);
    newLabelsObj = newLabels(newLabelSum>0,:);
    
    % All labels from all images. Many project to the same object. Subtract
    % 1 for the background class
    nLabels = numel(unique(newLabelsObj))-1;
    
    % Establish equivalence between labels from different images
    equivalence = zeros(nLabels,nLabels);
    NObjPoints = size(newLabelsObj,1);
    for i=1:NObjPoints
        % Get unique labels
        uniq = unique(newLabelsObj(i,:));
        uniq = uniq(uniq>0);
        
        % Equivalent labels are added to the adjacency matrix
        [p,q] = meshgrid(uniq, uniq);
        equivalence(p(:),q(:)) = 1;
    end
    
    % Get the connected components of the equivalence matrix -> unique
    % objects
    comps = components(equivalence);
    
    
    % Transform all labels into the ID of the connected component
    labSize = size(newLabels);
    newLabelsVec = newLabels(:);
    newLabelsVec(newLabelsVec>0) = comps(newLabelsVec(newLabelsVec>0));
    newLabels = reshape(newLabelsVec,labSize);
    
    % Select the mode of the labeling
    newLabels(newLabels==0)=NaN;
    newLabels_agreed = mode(newLabels,2);

    % Remove empty IDs
    [~,~,newLabels_uniq]  = unique(newLabels_agreed);
    newLabels_uniq(isnan(newLabels_agreed)) = NaN;

    
    fprintf('Exporting the point cloud...\n');
    ExportPointCloudLabels(['models/out_ID_' num2str(targetLabel) '.ply'],points,normals,newLabels_uniq);
end