function labels = Image2Label(image)

    if size(image,3)~=1
      labels = zeros(size(image,1),size(image,2));

      labels((image(:,:,1)==255) & (image(:,:,2)==0) & (image(:,:,3)==0)) = 1;%Window
      labels((image(:,:,1)==255) & (image(:,:,2)==255) & (image(:,:,3)==0)) = 2;%Wall
      labels((image(:,:,1)==128) & (image(:,:,2)==0) & (image(:,:,3)==255)) = 3;%Balcony
      labels((image(:,:,1)==255) & (image(:,:,2)==128) & (image(:,:,3)==0)) = 4;%Door
      labels((image(:,:,1)==0) & (image(:,:,2)==0) & (image(:,:,3)==255)) = 5;%Roof
      labels((image(:,:,1)==128) & (image(:,:,2)==255) & (image(:,:,3)==255)) = 6;%Sky
      labels((image(:,:,1)==0) & (image(:,:,2)==255) & (image(:,:,3)==0)) = 7;%Shop
    else
        labels = zeros(size(image,1),1);

      labels((image(:,1)==255) & (image(:,2)==0) & (image(:,3)==0)) = 1;%Window
      labels((image(:,1)==255) & (image(:,2)==255) & (image(:,3)==0)) = 2;%Wall
      labels((image(:,1)==128) & (image(:,2)==0) & (image(:,3)==255)) = 3;%Balcony
      labels((image(:,1)==255) & (image(:,2)==128) & (image(:,3)==0)) = 4;%Door
      labels((image(:,1)==0) & (image(:,2)==0) & (image(:,3)==255)) = 5;%Roof
      labels((image(:,1)==128) & (image(:,2)==255) & (image(:,3)==255)) = 6;%Sky
      labels((image(:,1)==0) & (image(:,2)==255) & (image(:,3)==0)) = 7;%Shop

    end
end