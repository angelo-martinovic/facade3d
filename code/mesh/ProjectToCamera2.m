function projection=ProjectToCamera2(points,values,camera,colormap)

    width = floor(camera.principalPoint(1)*2);
    height = floor(camera.principalPoint(2)*2);
    
%     depthImageSum = zeros(height,width);
%     depthImageCount = zeros(height,width);
  
%     projPoints = [];
%     projValues = [];
%     
%     
%     for i=1:length(points)
%        point = [points(i,:) 1];
%        
%        t = camera.P * point';
%        
%        t = t/t(3);
%         
%        %fprintf('%f %f\n',t(1),t(2));
%        x = round(t(1));
%        y = round(t(2));
%        
%        if x<1 || x>width
%            %fprintf('x:%f\n',x);
%            continue;
%        end
%        if y<1 || y>height
%            %fprintf('y:%f\n',y);
%            continue;
%        end
%        
%        projPoints = [projPoints; y x];
%        projValues = [projValues; values(i)];
%     end

    if size(points,1)==3
        points = points';
    end
    points = [points ones(length(points),1)];
    
    projPoints = camera.P * points';
    
    t=bsxfun(@rdivide,projPoints,projPoints(3,:));
    
    x = round(t(1,:));
    y = round(t(2,:));
    
    visibleMask = (x>=1 & x<=width & y>=1 & y<=height);
    
    xv = x(visibleMask);
    yv = y(visibleMask);
    
    projPoints = [yv' xv'];
    projValues = values(visibleMask==1);
    
    
    projection = zeros(height,width,1);
    [ii,jj] = find(projection==0);
    
    imagePoints = [ii jj];
    
    if isempty(projPoints)
        imValues = zeros(height,width);
    else
        idx = knnsearch(projPoints,imagePoints);

        imValues = projValues(idx);
        imValues = reshape(imValues,height,width);
    end
    
%     imValues = imrotate(imValues,-90);
    imValues = fliplr(imValues');
    
    projection = Label2Image(imValues,colormap);
    
end