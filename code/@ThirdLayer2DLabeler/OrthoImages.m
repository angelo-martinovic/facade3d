function OrthoImages(obj)

    facadeIDs = obj.GetFacadeIDs();
    [camerapos,cameras] = obj.GetCameraPos();
    splitDir = [obj.dirName 'work/pcl/split/' obj.splitName '/'];
    
    tic;
    for i=1:length(facadeIDs)
        facadeID = facadeIDs(i);
        
        fprintf('----\nProcessing facade %d (%d of %d) ...\n----',facadeID,i,length(facadeIDs));

        % Original colors
        [~,~,origColors] = ReadPCLFromPly([splitDir obj.modelName '_split_' num2str(facadeID) '_colors.ply']);

        % Potentials
        ss = load([splitDir obj.modelName '_split_' num2str(facadeID) '_potentials.mat']); %loads vertexSubsetPotentials
        vertexSubsetPotentials = ss.vertexSubsetPotentials;

        % Labeling
        [points,~,labelColors] = ReadPCLFromPly([splitDir obj.modelName '_split_' num2str(facadeID) '_labeling.ply']);

        % Facade plane
        ss=load([splitDir obj.modelName '_split_' num2str(facadeID) '_plane.mat']);
        plane = ss.plane;
        
        v = bsxfun(@minus,points,plane.p');    % vectors from points to plane origin
        dist = v * plane.n;        % distances from points to plane along normal
        projPoints = points - dist*plane.n';

        if size(projPoints,1)~=3
           projPoints = projPoints';
        end
       
        if isempty(plane)
            error('No main plane found! Ortho-rectification cannot continue.');
        end
  
        plane.b = [min(projPoints(1,:)) max(projPoints(1,:)) min(projPoints(2,:)) max(projPoints(2,:)) min(projPoints(3,:)) max(projPoints(3,:))];

        g = ss.g;
       
        bestHP = 0;
        bestG = ss.g;
        % Fine tune the gravity vector
        if obj.fineTuneGravityVector
            for xvec=0%-0.05:0.01:0.05
                for zvec=-0.05:0.01:0.05
                    yvec = sqrt(1-xvec*xvec-zvec*zvec);
                    % Modified gravity vector
                    g=[xvec yvec zvec];

                    % Tentative ortho image
                    [orthoImage,~] = ProjectToOrtho(plane,g,projPoints,origColors);
                  
                    % Get edge information, project on the vertical line
                    bw = edge(rgb2gray(orthoImage));
                    hp = sum(bw,2)/size(bw,2);

                    % Optimize for the highest value of the projection
                    % (which happens when image edges are aligned with the
                    % horizontal direction)
                    hp= sort(hp,'descend');
                    curHP = mean(hp(1:10));
                    fprintf('xvec:%f,zvec:%f,curHP:%f\n',xvec,zvec,curHP);

                    if curHP>bestHP
                        bestHP = curHP;
                        bestG = g;
                    end
                end
            end
           g = bestG;
           save([splitDir obj.modelName '_split_' num2str(facadeID) '_plane.mat'],'plane','g');
        end
        
       % Get the ortho labeling, ortho potentials, and 3D positions of
       % ortho image
       [orthoImageOld,iPointsR] = ProjectToOrtho(plane,g,projPoints,origColors); 
       [orthoLabel,~] = ProjectToOrtho(plane,g,projPoints,labelColors);
       [orthoPotentials,~] = ProjectToOrtho(plane,g,projPoints,vertexSubsetPotentials);
     
       %% Create the ortho image by averaging the projected colors from different cameras
       % (Provides better quality than the nearest neighbor labeling from
       % the point cloud)

       % Get nc closest cameras
       nc = 10;
       idx = knnsearch(camerapos,plane.p','K',nc);
    
       colorsPerPoint = zeros(length(iPointsR),nc,3);    
       % For each image
       fprintf('Projecting %d images onto the ortho image points\n',nc);
       for j=1:nc
            camIdx = idx(j);

            height = cameras{camIdx}.principalPoint(2)*2;
            width = cameras{camIdx}.principalPoint(1)*2;

            imgName = cameras{camIdx}.originalImageFilename(1:end-4);
            imageFilename = [obj.dirName 'image/' imgName '.jpg'];

            if exist(imageFilename,'file')

                origImg = imread(imageFilename);

                if height==size(origImg,2) && width==size(origImg,1)
                    origImg = imrotate(origImg,90);
                else
                    error('Camera-image size mismatch!');
                end

                % Backproject the colors
                for c=1:3
                    colorsPerPoint(:,j,c) = BackProjectLabeling(iPointsR,origImg(:,:,c),cameras{camIdx});
                end

                fprintf('o');
            else
                fprintf('x');
                continue;
            end

       end

       colors = squeeze(sum(colorsPerPoint,2)./sum(colorsPerPoint~=0,2))';
       colors(isnan(colors)) = 0;
       colors = round(colors');
     
       % Ortho - colors
       newColors = colors/255;
       orthoImage = reshape(newColors,[size(orthoImageOld,1),size(orthoImageOld,2),3]);
       orthoImage = imrotate(orthoImage,180);
       orthoImage = fliplr(orthoImage);
       figure(200);imagesc(orthoImage);axis equal;drawnow;
       imwrite(orthoImage,[splitDir obj.modelName '_split_' num2str(facadeID)  '_ortho_colors.png']);
        
       figure(300);imagesc(orthoLabel);axis equal;drawnow;
       imwrite(orthoLabel,[splitDir obj.modelName '_split_' num2str(facadeID)  '_ortho_labeling.png']);

       figure(400);imagesc(orthoPotentials(:,:,1));axis equal;drawnow;
       save([splitDir obj.modelName '_split_' num2str(facadeID)  '_ortho_potentials.mat'],'orthoPotentials');

       
    end
    orthoTime = toc;
    fprintf('Elapsed time: %d seconds.\n',orthoTime);
end


function [orthoImage,iPointsR] = ProjectToOrtho(plane,g,projPoints,origColors)
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

    R = [x';y';z'];

    pointsR = R*projPoints;
    %         figure;scatter(pointsR(1,:), pointsR(2,:),[],'r');

    % Transform points to the translated system
    pointsT = bsxfun(@minus,pointsR,o);

    % Get rectangle bounds
    minX = min(pointsT(1,:)); maxX = max(pointsT(1,:));
    minY = min(pointsT(2,:)); maxY = max(pointsT(2,:));
    meanZ = mean(pointsT(3,:));

    %                     b = [minX maxX minY maxY min(pointsT(3,:)) max(pointsT(3,:))];
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

    [idx,~] = knnsearch(pointsT',iPoints');
        
    % Ortho - colors
    newColors = origColors(idx,:)/255;
    orthoImage = reshape(newColors,[size(xx,1),size(xx,2),size(newColors,2)]);
    orthoImage = imrotate(orthoImage,180);
    orthoImage = fliplr(orthoImage);
        
    % Transform points from the translated system
    ipointsT = bsxfun(@minus,iPoints,-o);
        
    iPointsR = R\ipointsT;
end