function fig = VisualizeCameras(points,figIndex)
   
    if nargin<3
        figIndex = 1;
    end
   
    
    points = single(points);
    X = points(:,1);
    Y = points(:,2);
    Z = points(:,3);

    fig=figure(figIndex);
    skip = 1;

    hold on;
    scatter3(X(1:skip:end),Y(1:skip:end),Z(1:skip:end),50*ones(length(X(1:skip:end)),1),'r','fill');
    axis equal;

end
