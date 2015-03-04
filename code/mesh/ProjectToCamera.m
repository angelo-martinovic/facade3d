function image=ProjectToCamera(points,colors,P)

    height = 600;
    width = 800;
    
    image = zeros(height,width,3);
    for i=1:length(points)
       point = [points(i,:) 1];
       
       t = P * point';
       
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
       
       c1 = colors(i,1);
       c2 = colors(i,2);
       c3 = colors(i,3);
       
       psize = 4;
       
       ymin = y-psize/2;if ymin<1,ymin=1;end
       ymax = y+psize/2;if ymax>height,ymax=height;end
       
       xmin = x-psize/2;if xmin<1,xmin=1;end
       xmax = x+psize/2;if xmax>width,xmax=width;end
        
      
       
       image(ymin:ymax,xmin:xmax,1) = c1;
       image(ymin:ymax,xmin:xmax,2) = c2;
       image(ymin:ymax,xmin:xmax,3) = c3;
    end
    
    image = imrotate(image,-90);
    for i=1:3
        image2(:,:,i) = fliplr(image(:,:,i)');
    end
    image=image2;
%     figure(1);imagesc(image);
    

end