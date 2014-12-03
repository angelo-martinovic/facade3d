function ReassemblePointCloud(obj)
    % Load full mesh

    points = [];
    colors = [];

    splitDir = [obj.dirName 'work/pcl/split/' obj.baseName '_' obj.splitName '/'];
    ss = load([splitDir obj.baseName '_facadeIDs.mat']);
    facadeIDs = ss.facadeIDs';

    tic;
    pb = ProgressBar(length(facadeIDs));
    for i=1:length(facadeIDs)
        facadeID = facadeIDs(i);

        if facadeID==0
            pb.progress;
            continue;
        end

        filename = [splitDir obj.baseName '_split_' num2str(facadeID) '_labeling_layer3.ply'];
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

    fprintf('Loading the full mesh...');
    [pointsFull,~,colorsFull]=ReadPCLFromPly([obj.dirName obj.meshName]);
    fprintf('Done!\n');
    
    fprintf('KNN search...');
    idx = knnsearch(pointsFull,points);
    fprintf('Done!\n');

    colorsNew = zeros(size(colorsFull));
    colorsNew(idx,:) = colors;

    colorsMixed = round(mean(cat(3,colorsNew,colorsFull),3));

    fprintf('Exporting mesh...');
    ExportMesh([obj.dirName 'work/pcl/models/' obj.baseName '_'  obj.splitName '_2Dlayer3.ply'],pointsFull,[],colorsNew,[],[]);
    ExportMesh([obj.dirName 'work/pcl/models/' obj.baseName '_'  obj.splitName '_2Dlayer3_overlay.ply'],pointsFull,[],colorsMixed,[],[]);

    reassembleTime = toc;
    fprintf('Elapsed %d seconds.\n',reassembleTime);
end