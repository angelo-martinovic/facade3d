function img_out = RadialUndistort(img,camera)

    img_out = zeros(size(img));
    
    h = size(img,1);
    w = size(img,2);
    
    f2_inv = 1/(camera.focalLength*camera.focalLength);
    
    for y=1:h
        for x=1:w
            x_c = x-w/2;
            y_c = y-h/2;
            
            r2 = (x_c^2 + y_c^2)*f2_inv;
            factor = 1-camera.normRadDistortion*r2;
            
            x_c = x_c*factor;
            y_c = y_c*factor;
            
            x_c = x_c + w/2;
            y_c = y_c + h/2;
            
            if (x_c>=1 && x_c<=w && y_c>=1 && y_c<=h)
                if size(img,3)>1
                   img_out(y,x,1)  = img(round(y_c),round(x_c),1);
                   img_out(y,x,2)  = img(round(y_c),round(x_c),2);
                   img_out(y,x,3)  = img(round(y_c),round(x_c),3);
                else
                   img_out(y,x)  = img(round(y_c),round(x_c));
                end
            end
            
        end
    end
    img_out = uint8(img_out);
%     figure(1);imshow(img);title('Original');
%     figure(2);imshow(img_out);title('Undistorted');
end

