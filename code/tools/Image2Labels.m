function labels = Image2Labels(image,cm)

    colors = reshape(image,[size(image,1)*size(image,2) 3]);
    labels = Colors2Labels(colors,cm);
    labels = reshape(labels,[size(image,1) size(image,2)]);
%   labels = zeros(size(colors,1),1);
% 
%   labels((colors(:,1)==255) & (colors(:,2)==0) & (colors(:,3)==0)) = 1;%Window
%   labels((colors(:,1)==255) & (colors(:,2)==255) & (colors(:,3)==0)) = 2;%Wall
%   labels((colors(:,1)==128) & (colors(:,2)==0) & (colors(:,3)==255)) = 3;%Balcony
%   labels((colors(:,1)==255) & (colors(:,2)==128) & (colors(:,3)==0)) = 4;%Door
%   labels((colors(:,1)==0) & (colors(:,2)==0) & (colors(:,3)==255)) = 5;%Roof
%   labels((colors(:,1)==128) & (colors(:,2)==255) & (colors(:,3)==255)) = 6;%Sky
%   labels((colors(:,1)==0) & (colors(:,2)==255) & (colors(:,3)==0)) = 7;%Shop
  
end