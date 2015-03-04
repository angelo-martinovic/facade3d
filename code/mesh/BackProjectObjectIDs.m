function newLabels=BackProjectObjectIDs(points,image,camera)

    height = floor(camera.principalPoint(2)*2);
    width = floor(camera.principalPoint(1)*2);
    newLabels = zeros(length(points),1);
    
%     usedImage = zeros(size(image));
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
       
%        usedImage(y,x) = usedImage(y,x)+1;
               
       id = image(y,x);

       newLabels(i) = id;

      
    end

   % figure;imagesc(usedImage);
end

