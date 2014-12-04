% Takes a rectified image(one channel) and the homography used to rectify it, and
% "unrectifies" it.

% img - the original unrectified image we want to project to
% imw - the labeling of the rectified image
% h - the file which specifies the homography between the two images
function unrectifiedImage = rewarp(img, imw, h, defValue, type)
  
    if nargin<4
        defValue = 0;
    end
    
    if nargin<5
        type = 'nearest';
    end
    
    h = reshape(h,3,3);
    h2 = h';

    [MX, MY] = meshgrid(1:size(img,2), 1:size(img,1));
    X = [ MX(:)' ; MY(:)' ; ones(1,numel(MX)) ];
    Y = bsxfun(@rdivide, h2* X, h2(3,:) * X);
    

    Ix= round(Y(2,:));
    Iy= round(Y(1,:));
    
    unrectifiedImage =  interp2(1:size(imw,2), 1:size(imw,1), imw(:,:), Iy, Ix, type, defValue);
    
    unrectifiedImage =  reshape(unrectifiedImage, size(img,1), size(img,2));

    assert ( ~any(any(isnan(unrectifiedImage))) );

end