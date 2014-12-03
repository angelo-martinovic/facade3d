% Load the mesh
[pointsGT,~,colorsGT,facesGT,facesColorsGT]=ReadMeshFromPly('/esat/sadr/amartino/mongeSplitSub2/Paris_RueMonge_subset30_800px_meshAvImgCol_simplified10_mesh_split2.ply');

% Index maps
mapDirName = '/esat/sadr/amartino/mongeSplitSub2/Paris_RueMonge_subset30_800px_meshAvImgCol_simplified10/';
mapImageFilenames = dir([mapDirName '*.txt']);
nMaps = length(mapImageFilenames);

% Labelings
labelDirName = '/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/visualizeRotated/output/svm_monge428_newTrain/';

nClasses = 7;
nFaces = size(facesGT,1);
meshLabelMapping = zeros(nFaces,nClasses);
% For every image
for i=1:nMaps
    fprintf('.');
    if mod(i,10)==0
        fprintf('%d\n',i);
    end
        
    % Get index map (hayko)
    iMap = dlmread([mapDirName mapImageFilenames(i).name]);
    
    % Get labeling map (us)
    lMapStruct = load([labelDirName mapImageFilenames(1).name(end-20:end-13) '.2Dpotentials.mat']);
    lMap = lMapStruct.labeling;
    
    lMap = imrotate(lMap,-90);
    
    % Label mesh faces
    observedFaces = unique(iMap);
    observedFaces = observedFaces(2:end);   % eliminate background
    nFaces = size(observedFaces,1);
    
    for j=1:nFaces
       meshLabelMapping(j,:) = meshLabelMapping(j,:) + hist(lMap(iMap==observedFaces(j)),1:nClasses);
       
    end
    
end

[~,meshLabeling] = max(meshLabelMapping,[],2);
% Save the labeled mesh