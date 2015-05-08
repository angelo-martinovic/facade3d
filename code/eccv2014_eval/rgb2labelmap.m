function [im_label, im_labeL_rgb] = rgb2labelmap(im_labeL_rgb, labelmap)
% [im_label, im_labeL_rgb] = cvl_rgb2labelmap(im_labeL_rgb, labelmap)
%   Convert a RGB color labelmap into an index-based label map.
%
% author: hayko riemenschneider, 2014
%
% see also colormap_zurich2rgb.m, label2rgb

im_labeL_rgb = im2double(im_labeL_rgb);

n = size(im_labeL_rgb,1)*size(im_labeL_rgb,2);
[~, idx]= min(pdist2(reshape(im_labeL_rgb, n,3), labelmap,'euclidean'),[],2);
im_label = uint8(reshape(idx-1, size(im_labeL_rgb,1),size(im_labeL_rgb,2),1));

% if 2nd return parameter
if(nargout>1)
    im_labeL_rgb = label2rgb(im_label, labelmap);
end
