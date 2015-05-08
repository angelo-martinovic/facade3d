function data_raw = mesh_collect_raw(im_raw, im_idx,numFaces)
% data_raw = mesh_collect_raw(im_raw, im_idx,numFaces)
%  Collect data from RGB images into a triangle indexed data structure
%
% author: hayko riemenschneider, 2014

numLabels = size(im_raw,3);

cntperindex = accumarray (im_idx(:),1, [numFaces+1 1]);
data_raw = zeros(numLabels, numFaces);

for labidx = 1:numLabels
    values = im_raw(:,:,labidx);
    sumperindex = accumarray (im_idx(:),values(:), [numFaces+1 1]);
    data_raw(labidx,:) = sumperindex(2:end) ./ max(1,cntperindex(2:end)); % LATER: GET RID OF +1
end


end