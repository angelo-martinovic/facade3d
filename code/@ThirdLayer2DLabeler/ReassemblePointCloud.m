function ReassemblePointCloud(obj)
    % Load full mesh
    points = [];
    colors = [];

    facadeIDs = obj.GetFacadeIDs();

    tic;
    pb = ProgressBar(length(facadeIDs));
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

        pb.progress;
    end
    pb.stop;

    fprintf('Loading the full point cloud...');
    [pointsFull,~,colorsFull] = ReadPCLFromPly(get_adr('pcl',obj.config));
    fprintf('Done!\n');
    
    fprintf('KNN search...');
    idx = knnsearch(pointsFull,points);
    fprintf('Done!\n');

    colorsNew = zeros(size(colorsFull));
    colorsNew(idx,:) = colors;

    colorsMixed = round(mean(cat(3,colorsNew,colorsFull),3));

    fprintf('Exporting mesh...');
    ExportMesh(get_adr('3D_L3_Ortho2D_labeling',obj.config,obj.splitName,facadeID),pointsFull,[],colorsNew,[],[]);
    ExportMesh(get_adr('3D_L3_Ortho2D_labeling_overlay',obj.config,obj.splitName,facadeID),pointsFull,[],colorsMixed,[],[]);

    reassembleTime = toc;
    fprintf('Elapsed %d seconds.\n',reassembleTime);
end