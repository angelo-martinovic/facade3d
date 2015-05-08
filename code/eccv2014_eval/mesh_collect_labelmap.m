function data_test = mesh_collect_labelmap(im_test_rgb, im_idx, cm, numFaces)
% data_test = mesh_collect_labelmap(im_test_rgb, im_idx, cm, numFaces)
%  Collect data from label images into a triangle indexed data structure
%
% author: hayko riemenschneider, 2014

    im_test = rgb2labelmap(im_test_rgb,cm);
    numLabels = size(cm,1);
    data_test = zeros(numLabels, numFaces);

    for labidx = 0:numLabels-1
        values = im_test==labidx;
        sumperindex = accumarray (im_idx(:),values(:), [numFaces+1 1]);
        data_test(labidx+1,:) = sumperindex(2:end);% LATER: GET RID OF +1
    end
        
end