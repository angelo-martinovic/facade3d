function FitPlanes(obj)
    fprintf('Fitting planes to each facade...\n');
    camerapos = obj.GetCameraPos();
    facadeIDs = obj.GetFacadeIDs();
    
    N = length(facadeIDs);
    planes = cell(N,1);
%     gravityVectors = cell(N,1);
    tic;
    % For each facade
    for i=1:N
        facadeID = facadeIDs(i);
        
        fprintf('----\nProcessing facade %d (%d of %d) ...\n----',facadeID,i,N);

        % Get facade points
        [points,~,~] = ReadPCLFromPly(get_adr('splitLabeling',obj.config,obj.splitName,num2str(facadeID)));

        % Fit a plane to the facade
        fprintf('Fitting plane ...');
        plane = FitPlane(points);
        fprintf('Done.\n');

        if isempty(plane)
            error('No main plane found!');
        end

        plane.n = plane.n/norm(plane.n);

        % Project points to the plane
        v2 = bsxfun(@minus,camerapos,plane.p');    % vectors from points to plane origin
        dist2 = v2 * plane.n;        % distances from cameras to plane along normal

        if sum(dist2<0) > sum(dist2>0)
            % More cameras are behind the plane than in front - flip the normal
            plane.n = - plane.n;
        end

        v = bsxfun(@minus,points,plane.p');    % vectors from points to plane origin
        dist = v * plane.n;        % distances from points to plane along normal
        projPoints = points - dist*plane.n';

        if size(projPoints,1)~=3
           projPoints = projPoints';
        end

        % Update the plane bounding box
        plane.b = [min(projPoints(1,:)) max(projPoints(1,:)) min(projPoints(2,:)) max(projPoints(2,:)) min(projPoints(3,:)) max(projPoints(3,:))];

        planes{i} = plane;
       
       % New code, not so precise
%        gravityVectors{i} = obj.GetGravityVector(facadeID,plane.n);

    end
    
    % Gravity vector estimation
    % Old code, works better
    g = GetGravityVector([planes{:}]);
    
    % Save plane and gravity vector information
    for i = 1:N
        facadeID = facadeIDs(i);
        if facadeID==0
            continue;
        end
        plane = planes{i}; %#ok<NASGU>
%         g = gravityVectors{i}; %#ok<NASGU>
        save(get_adr('splitPlane',obj.config,obj.splitName,num2str(facadeID)),'plane','g');
    end
    fitPlaneTime = toc;
    fprintf('Elapsed time: %d\n',fitPlaneTime);
end
