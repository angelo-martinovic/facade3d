function FitPlanes(obj)
    dl = DispatchingLogger.getInstance();
    dl.Log(VerbosityLevel.Info,sprintf('Fitting a plane to each facade...\n'));

    camerapos = obj.GetCameraPos();
    facadeIDs = obj.GetFacadeIDs();
    
    N = length(facadeIDs);
    planes = cell(N,1);
    tic;
    % For each facade
    for i=1:N
        facadeID = facadeIDs(i);
        if facadeID==0, continue; end;
        dl.Log(VerbosityLevel.Debug,...
            sprintf(' - Processing facade %d (%d of %d) ...\n',facadeID,i,N));

        % Get facade points
        [points,~,~] = ReadPCLFromPly(get_adr('splitLabeling',obj.config,obj.splitName,num2str(facadeID)));

        % Fit a plane to the facade
        dl.Log(VerbosityLevel.Debug,sprintf(' - - Fitting plane ...\n'));
        plane = FitPlane(points);
        dl.Log(VerbosityLevel.Debug,sprintf(' - - Done.\n'));

        if isempty(plane)
            dl.Log(VerbosityLevel.Error,sprintf(' - No main plane found!\n'));
            fatal();
        end

        % Normalize plane normal
        plane.n = plane.n/norm(plane.n);

        % Project points to the plane
        v2 = bsxfun(@minus,camerapos,plane.p');    % vectors from points to plane origin
        dist2 = v2 * plane.n;        % distances from cameras to plane along normal

        % If more cameras are behind the plane than in front - flip the normal
        if sum(dist2<0) > sum(dist2>0)
            plane.n = - plane.n;
        end

        v = bsxfun(@minus,points,plane.p');    % vectors from points to plane origin
        dist = v * plane.n;        % distances from points to plane along normal
        projPoints = points - dist*plane.n';

        % Sanity check
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
    g = GetGravityVector([planes{:}]); %#ok<NASGU>
    
    % Save plane and gravity vector information
    for i = 1:N
        facadeID = facadeIDs(i);
        if facadeID==0, continue; end;
        plane = planes{i}; %#ok<NASGU>
%         g = gravityVectors{i}; %#ok<NASGU>
        save(get_adr('splitPlane',obj.config,obj.splitName,num2str(facadeID)),'plane','g');
    end
    dl.Log(VerbosityLevel.Info,sprintf('Done. Elapsed time: %.2f seconds.\n',toc));
end
