function projection=ProjectToCamera2(points,values,camera)

    width = floor(camera.principalPoint(1)*2);
    height = floor(camera.principalPoint(2)*2);
    
%     depthImageSum = zeros(height,width);
%     depthImageCount = zeros(height,width);
  
    projPoints = [];
    projValues = [];
    for i=1:length(points)
       point = [points(i,:) 1];
       
       t = camera.P * point';
       
       t = t/t(3);
        
       %fprintf('%f %f\n',t(1),t(2));
       x = round(t(1));
       y = round(t(2));
       
       if x<1 || x>width
           %fprintf('x:%f\n',x);
           continue;
       end
       if y<1 || y>height
           %fprintf('y:%f\n',y);
           continue;
       end
       
       projPoints = [projPoints; y x];
       projValues = [projValues; values(i)];
    end

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
    
    projection = Label2Image(imValues,HaussmannColormap());
    
end