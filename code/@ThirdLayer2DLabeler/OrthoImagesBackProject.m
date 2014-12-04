function OrthoImagesBackProject(obj)

%     g = GetGravityVector();

    splitDir = [obj.dirName 'work/pcl/split/' obj.baseName '_' obj.splitName '/'];
    ss = load([splitDir obj.baseName '_facadeIDs.mat']);
    facadeIDs = ss.facadeIDs';

    tic;
    for i=1:length(facadeIDs)
        facadeID = facadeIDs(i);

        if facadeID==0
            continue;
        end
%         g = obj.GetGravityVector(facadeID);

        fprintf('----\nProcessing facade %d (%d of %d) ...\n----',facadeID,i,length(facadeIDs));

        % Labeling
        [points,~,~] = ReadPCLFromPly([splitDir obj.baseName '_split_' num2str(facadeID) '_labeling.ply']);

        % Facade plane
        ss=load([splitDir obj.baseName '_split_' num2str(facadeID) '_plane.mat']);
        plane = ss.plane;
        g= ss.g;

        if isempty(plane)
            error('No main plane found! Ortho-rectification cannot continue.');
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
        orthoImageFilename = [splitDir obj.baseName '_split_' num2str(facadeID) '_ortho_labeling_layer3.png'];
        if ~exist(orthoImageFilename,'file')
            warning('%s does not exist.\n',orthoImageFilename);
            continue;
        end
        orthoImage = imread(orthoImageFilename);
        orthoImage = fliplr(orthoImage);
        orthoImage = imrotate(orthoImage,180);
        orthoImageColors = reshape(orthoImage,[size(orthoImage,1)*size(orthoImage,2),3]);

        assert(isequal(size(orthoImageColors,1),size(iPoints,2)),...
            'Read ortho image size: %d %d, sampled points: %d %d',size(orthoImage,1),size(orthoImage,2),iHeight,iWidth);

        % Project the points to the world coordinate system
        iPointsT = bsxfun(@plus,iPoints,o);
    %         Rinv = inv(R);
        iPointsR = R\iPointsT;

        % Find the closest pcl points
        idx = knnsearch(iPointsR',points);

        % Label the pcl points with the appropriate ortho colors
        pointSubsetColors = orthoImageColors(idx,:);

        ExportMesh([splitDir obj.baseName '_split_' num2str(facadeID)  '_labeling_layer3.ply'],points,[],pointSubsetColors,[],[]);
    end
    orthoTime = toc;
    fprintf('Elapsed time: %d seconds.\n',orthoTime);
end