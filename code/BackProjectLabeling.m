function [newLabels,nonZeroIndices]=BackProjectLabeling(points,image,camera)

    height = floor(camera.principalPoint(2)*2);
    width = floor(camera.principalPoint(1)*2);

    newLabels = zeros(length(points),size(image,3));
       
    if size(points,1)==3
        points = points';
    end
    points = [points ones(length(points),1)];
    
    projPoints = camera.P * points';
    
    t=bsxfun(@rdivide,projPoints,projPoints(3,:));
    
    x = round(t(1,:));
    y = round(t(2,:));
    
    visibleMask = (x>=1 & x<=width & y>=1 & y<=height);
    visibleIndices = find(visibleMask);
    
    xv = x(visibleMask);
    yv = y(visibleMask);
    
    
    for i=1:size(image,3)
        projection = image(sub2ind(size(image),yv,xv,repmat(i,size(yv))));

        nonzero = (projection>0);
        nonZeroIndices = visibleIndices(nonzero);

        visibleNonZeroLabels = projection(nonzero);
   
        newLabels(nonZeroIndices,i) = visibleNonZeroLabels;
    end
    
end

