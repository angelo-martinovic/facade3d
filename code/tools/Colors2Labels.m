function labels = Colors2Labels(colors,cm)

    labels = knnsearch(cm,colors)-1;
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