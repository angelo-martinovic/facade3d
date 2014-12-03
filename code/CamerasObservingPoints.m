function cameraIDs = CamerasObservingPoints(cameras,points)   

    cameraIDs = [];
    
    points = [points ones(length(points),1)];
    
    for i=1:length(cameras)
        camera = cameras{i};
        
        height = floor(camera.principalPoint(2)*2);
        width = floor(camera.principalPoint(1)*2);
        
        projPoints = camera.P * points';

        t=bsxfun(@rdivide,projPoints,projPoints(3,:));

        x = round(t(1,:));
        y = round(t(2,:));

        visibleMask = (x>=1 & x<=width & y>=1 & y<=height);
        visibleIndices = find(visibleMask);
        
        if ~isempty(visibleIndices)
            cameraIDs(end+1)= i;
        end
    end
    
end