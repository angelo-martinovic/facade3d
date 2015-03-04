function image = Label2Image(label,cm)

    image = cm(label(:)+1,:);
    image = reshape(image,[size(label) 3]);
    
    image = uint8(256*image);
    
% height = size(label,1);
% width = size(label,2);
% 
% image = zeros(height,width,3);
% imageR = zeros(height,width);imageG = zeros(height,width);imageB = zeros(height,width);
% 
%         imageR(label==1) = 255; imageG(label==1) = 0;    imageB(label==1) = 0; %win
%         imageR(label==2) = 255; imageG(label==2) = 255;  imageB(label==2) = 0;   %wall
%         imageR(label==3) = 128; imageG(label==3) = 0;    imageB(label==3) = 255;     %balc
%         imageR(label==4) = 255; imageG(label==4) = 128;  imageB(label==4) = 0;   %door
%         imageR(label==5) = 0;   imageG(label==5) = 0;    imageB(label==5) = 255; %roof
%         imageR(label==6) = 128; imageG(label==6) = 255;  imageB(label==6) = 255; %sky
%         imageR(label==7) = 0;   imageG(label==7) = 255;  imageB(label==7) = 0;  %shop
%         imageR(label==8) = 0;   imageG(label==8) = 0;    imageB(label==8) = 255;  %chimney
%         
% image(:,:,1) = imageR/255;
% image(:,:,2) = imageG/255;
% image(:,:,3) = imageB/255;
end
