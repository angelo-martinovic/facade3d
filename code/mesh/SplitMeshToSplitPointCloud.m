dirName = '/esat/sadr/amartino/monge428New/data/';

ss = load('/esat/sadr/amartino/mongeSplitSub2/Paris_RueMonge_part1_ruemonge_107_027_800px_meshAvImgCol_27x_GT_split_label.mat');
facadeLabels = ss.GT_split_label;
[points,~,~,faces,~]=ReadMeshFromPly('/esat/sadr/amartino/mongeSplitSub2/Paris_RueMonge_part1_ruemonge_107_027_800px_meshAvImgCol_27x_mesh_split2_binary.ply');

facadeIDs = unique(facadeLabels);
nFacades = size(facadeIDs,1);
splitLabels = zeros(size(points,1),1);
%%
for i=1:nFacades
    
    facadeID = facadeIDs(i);

    if facadeID==1 % void class
        continue;
    end
    
    % Get the part of the mesh corresponding to that facade
    faceSubset = faces(facadeLabels==facadeID,:);
    
    [vertexSubsetIndices,~,ic] = unique(faceSubset);
    faceSubsetTransformed = reshape(ic,size(faceSubset,1),3) ;   
    
    splitLabels(vertexSubsetIndices) = facadeID;
end

targetMeshName = 'fullRes_mesh';
[pointsTarget,~,~]=ReadPCLFromPly(['/esat/sadr/amartino/monge428New/data/' targetMeshName '_colors_normals.ply']);


sourceToTarget = knnsearch(points,pointsTarget);
splitLabelsTarget = splitLabels(sourceToTarget);

splitLabels = splitLabelsTarget;
save([dirName targetMeshName '_splitData.mat'],'splitLabels');
