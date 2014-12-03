%outImg - image you want to overlay
%name - output image name
%bgImage - image you are drawing over
%alpha - 0 - only background visible, 1 - only foreground visible
%mode - 1 for output of probability maps, 2 for segmentations
%selector - when mode==1, selects the RGB channel
%         - when mode==2, selects either the haussmann or eTrims colormap
function writeSegmentationToDisk(outImg,name,bgImage,alpha,mode,selector)
if nargin<5
    return
end
if nargin<6
    selector=1;
end

if alpha>1
    alpha=1;
elseif alpha<0
    alpha=0;
end

height = size(outImg,1);
width = size(outImg,2);

image = zeros(height,width,3);
imageR = zeros(height,width);imageG = zeros(height,width);imageB = zeros(height,width);
    
if (mode==1)
    maxVal = max(outImg(:));
    minVal = min(outImg(:));

    scaled = (outImg-minVal) / (maxVal-minVal) * 255;
    if (selector==1)
        imageR = scaled;
    elseif selector==2
        imageG = scaled;
    else
        imageB = scaled;
    end
elseif(mode==2)
    if (selector==1)
        %Haussmann colormap
        imageR(outImg==1) = 255; imageG(outImg==1) = 0;    imageB(outImg==1) = 0; %win
        imageR(outImg==2) = 255; imageG(outImg==2) = 255;  imageB(outImg==2) = 0;   %wall
        imageR(outImg==3) = 128; imageG(outImg==3) = 0;    imageB(outImg==3) = 255;     %balc
        imageR(outImg==4) = 255; imageG(outImg==4) = 128;  imageB(outImg==4) = 0;   %door
        imageR(outImg==5) = 0;   imageG(outImg==5) = 0;    imageB(outImg==5) = 255; %roof
        imageR(outImg==6) = 128; imageG(outImg==6) = 255;  imageB(outImg==6) = 255; %sky
        imageR(outImg==7) = 0;   imageG(outImg==7) = 255;  imageB(outImg==7) = 0;  %shop
        imageR(outImg==8) = 0;   imageG(outImg==8) = 0;    imageB(outImg==8) = 255;  %chimney
    else
        %eTrims colormap
        imageR(outImg==1) = 255; imageG(outImg==1) = 0;    imageB(outImg==1) = 0; %building
        imageR(outImg==2) = 255; imageG(outImg==2) = 255;  imageB(outImg==2) = 0;   %car
        imageR(outImg==3) = 128; imageG(outImg==3) = 0;    imageB(outImg==3) = 255;  %door
        imageR(outImg==4) = 255; imageG(outImg==4) = 128;  imageB(outImg==4) = 0;   %pavement
        imageR(outImg==5) = 128; imageG(outImg==5) = 128;  imageB(outImg==5) = 128; %road
        imageR(outImg==6) = 128; imageG(outImg==6) = 255;  imageB(outImg==6) = 255; %sky
        imageR(outImg==7) = 0;   imageG(outImg==7) = 255;  imageB(outImg==7) = 0;  %vegetation
        imageR(outImg==8) = 0;   imageG(outImg==8) = 0;    imageB(outImg==8) = 255;  %window
    end
    
   
end
image(:,:,1) = imageR;
image(:,:,2) = imageG;
image(:,:,3) = imageB;

imwrite(imlincomb(alpha, uint8(image), 1-alpha, bgImage),name);
end