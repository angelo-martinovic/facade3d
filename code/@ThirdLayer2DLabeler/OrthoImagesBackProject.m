function OrthoImagesBackProject(obj)
    dl = DispatchingLogger.getInstance();
    dl.Log(VerbosityLevel.Info,...
        sprintf('Back-projecting 3rd layer labelings to the point cloud...\n'));
        
    facadeIDs = obj.GetFacadeIDs();

    tic;
    for i=1:length(facadeIDs)
       facadeID = num2str(facadeIDs(i));

       dl.Log(VerbosityLevel.Debug,...
            sprintf(' - Processing facade ID %s (%d of %d) ...\n',facadeID,i,length(facadeIDs)));

       % Labeling
       [points,~,~] = ReadPCLFromPly(get_adr('splitLabeling',obj.config,obj.splitName,facadeID));

       % Facade plane
       ss=load(get_adr('splitPlane',obj.config,obj.splitName,facadeID));
       plane = ss.plane;
       g= ss.g;

       if isempty(plane)
           dl.Log(VerbosityLevel.Error,...
               sprintf(' - No main plane found! Ortho-rectification cannot continue.\n'));
           error('Critical error. Terminating.');
       end

       v = bsxfun(@minus,points,plane.p');    % vectors from points to plane origin
       dist = v * plane.n;        % distances from points to plane along normal
       projPoints = points - dist*plane.n';

       if size(projPoints,1)~=3
          projPoints = projPoints';
       end


       % Calculate the local coordinate system of the plane
       z = plane.n;
       y = -g/norm(g);        % Up-vector
       x = cross(y,z);  % Right-vector
       x = x/norm(x);

       if size(x,1)==1, x = x'; end
       if size(y,1)==1, y = y'; end
       if size(z,1)==1, z = z'; end

       % Origin
       o = [plane.b(1) plane.b(3) plane.b(5)]';

       % Find the rotation matrix between the world coord system and the 
       % plane-centered system
       R = [x';y';z'];

       pointsR = R*projPoints;

        % Transform points to the translated system
       pointsT = bsxfun(@minus,pointsR,o);

        % Get rectangle bounds
       minX = min(pointsT(1,:)); maxX = max(pointsT(1,:));
       minY = min(pointsT(2,:)); maxY = max(pointsT(2,:));
       meanZ = mean(pointsT(3,:));

       rHeight = maxY-minY;
       rWidth = maxX-minX;

       % Sample the rectangle
       iHeight = 800;
       iWidth = round( iHeight * (maxX-minX)/(maxY-minY) ) ;

       sampleIntervalY = rHeight/ (iHeight-1);
       sampleIntervalX = rWidth / (iWidth-1);
        
       % Ortho-image pixel positions
       [xx,yy] = meshgrid(minX:sampleIntervalX:maxX,minY:sampleIntervalY:maxY);
       iPoints=[xx(:) yy(:) repmat(meanZ,numel(xx),1)]';

       % Get the ortho-image
       orthoImageFilename = get_adr('orthoLabelingLayer3Img',obj.config,obj.splitName,facadeID);
       if ~exist(orthoImageFilename,'file')
           dl.Log(VerbosityLevel.Warning,...
               sprintf(' - Ortho labeling %s does not exist.\n',orthoImageFilename));
           continue;
       end
       orthoImage = imread(orthoImageFilename);
       orthoImage = fliplr(orthoImage);
       orthoImage = imrotate(orthoImage,180);
       orthoImageColors = reshape(orthoImage,[size(orthoImage,1)*size(orthoImage,2),3]);

       if ~isequal(size(orthoImageColors,1),size(iPoints,2))
           dl.Log(VerbosityLevel.Error,...
               sprintf(' - - Read ortho image size: %d %d, sampled points: %d %d',...
               size(orthoImage,1),size(orthoImage,2),iHeight,iWidth));
           error('Critical error. Terminating.');
       end

       % Project the points to the world coordinate system
       iPointsT = bsxfun(@plus,iPoints,o);
       iPointsR = R\iPointsT;

       % Find the closest pcl points
       idx = knnsearch(iPointsR',points);

       % Label the pcl points with the appropriate ortho colors
       pointSubsetColors = orthoImageColors(idx,:);

       ExportMesh(get_adr('orthoLabelingLayer3Ply',obj.config,obj.splitName,facadeID),points,[],pointSubsetColors,[],[]);
    end
    dl.Log(VerbosityLevel.Info,sprintf('Done. Elapsed time: %.2f seconds.\n',toc));
end