function depthImage=ProjectDepthToCamera(points,depths,camera)

    height = floor(camera.principalPoint(2)*2);
    width = floor(camera.principalPoint(1)*2);
    
    depthImageSum = zeros(height,width);
    depthImageCount = zeros(height,width);
  
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
       
       d = depths(i);

       psize = 8;
       
       ymin = y-psize/2;if ymin<1,ymin=1;end
       ymax = y+psize/2;if ymax>height,ymax=height;end
       
       xmin = x-psize/2;if xmin<1,xmin=1;end
       xmax = x+psize/2;if xmax>width,xmax=width;end
        
       depthImageSum(ymin:ymax,xmin:xmax) = d + depthImageSum(ymin:ymax,xmin:xmax);
       depthImageCount(ymin:ymax,xmin:xmax) = 1 + depthImageCount(ymin:ymax,xmin:xmax);
       
       % Override the old value
       % 1. if the new point has a smaller absolute value
       % 2. if there was no old value
%        if depthImage(y,x)>0 && abs(d)<abs(depthImage(y,x)) || depthImage(y,x)==0
%            depthImage(ymin:ymax,xmin:xmax) = d;
%        end
    end
    depthImage = depthImageSum./depthImageCount;
    depthImage(isnan(depthImage))=0;

    
%     depthImage = imrotate(depthImage,-90);
%    
%     depthImage = fliplr(depthImage');
    


end