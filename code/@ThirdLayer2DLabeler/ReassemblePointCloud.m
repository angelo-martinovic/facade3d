function ReassemblePointCloud(obj)
    % Load full mesh
    dl = DispatchingLogger.getInstance();
    dl.Log(VerbosityLevel.Info,...
        sprintf('Re-assembling the facade point clouds...\n'));
    points = [];
    colors = [];

    facadeIDs = obj.GetFacadeIDs();

    tic;
%     pb = ProgressBar(length(facadeIDs));
    for i=1:length(facadeIDs)
        facadeID = num2str(facadeIDs(i));

        filename = get_adr('orthoLabelingLayer3Ply',obj.config,obj.splitName,facadeID);
        if ~exist(filename,'file')
            warning([filename ' does not exist.']);
            continue;
        end

        [pointsPart,~,colorsPart]=ReadPCLFromPly(filename);

        points = [points; pointsPart];
        colors = [colors; colorsPart];

%         pb.progress;
    end
%     pb.stop;

    [pointsFull,~,colorsFull] = ReadPCLFromPly(get_adr('pcl',obj.config));
    
    dl.Log(VerbosityLevel.Debug,sprintf(' - KNN search...\n'));
    idx = knnsearch(pointsFull,points);
    dl.Log(VerbosityLevel.Debug,sprintf(' - Done!\n'));

    colorsNew = zeros(size(colorsFull));
    colorsNew(idx,:) = colors;

    colorsMixed = round(mean(cat(3,colorsNew,colorsFull),3));

    dl.Log(VerbosityLevel.Debug,sprintf(' - Exporting the labeled point cloud and overlay...\n'));
    ExportMesh(get_adr('3D_L3_Ortho2D_labeling',obj.config,obj.splitName,facadeID),pointsFull,[],colorsNew,[],[]);
    ExportMesh(get_adr('3D_L3_Ortho2D_labeling_overlay',obj.config,obj.splitName,facadeID),pointsFull,[],colorsMixed,[],[]);
    dl.Log(VerbosityLevel.Debug,sprintf(' - Done!\n'));
    
    dl.Log(VerbosityLevel.Info,sprintf('Done. Elapsed time: %.2f seconds.\n',toc));
end