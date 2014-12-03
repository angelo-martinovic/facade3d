function bbx = getMirroredBBx(win_boundingBoxes, x, width, im)
    %mirror only the ones left and right of the symmetry line
    mirroredr = win_boundingBoxes(:, win_boundingBoxes(3,:) < x-3);
    mirroredl = win_boundingBoxes(:, win_boundingBoxes(1,:) > x+3);
    tmpl = mirroredl;
    tmpr = mirroredr;
    mirroredl(1,:) = x+ (x-tmpl(3,:));
    mirroredl(3,:) = x + (x-tmpl(1,:)); 
    mirroredr(1,:) = x+ (x-tmpr(3,:));
    mirroredr(3,:) = x + (x-tmpr(1,:)); 
    mirroredl = mirroredl(:,mirroredl(1,:) > 0);
    mirroredr = mirroredr(:,mirroredr(3,:) <= width);
   
    
    %[mirroredl , dummy] = nms(win_boundingBoxes, mirroredl,0.5);
    %[mirroredr , dummy] = nms(win_boundingBoxes, mirroredr, 0.5);
    bbx = [mirroredl mirroredr];
    %figure(555); imagesc(im); axis image;
    %draw(win_boundingBoxes, 555, [1 0 0]);
    %draw(mirroredl, 555, [0 1 0]);
    %draw(mirroredr, 555, [0 1 0]);
end

function draw(p2,fignum, c)

    figure(fignum);axis image;   hold on

    for k=1:size(p2,2)

           x = round(p2(1,k));
           y = round(p2(2,k));
           w = round(p2(3,k)-p2(1,k));
           h = round(p2(4,k)-p2(2,k));
           X =[x, x+w, x+w,x+w,x+w,x, x,x];
           Y =[y, y,y, y+h, y+h,y+h, y+h, y];
           line(X,Y,'LineWidth',4,'Color',c);  
    end

end

