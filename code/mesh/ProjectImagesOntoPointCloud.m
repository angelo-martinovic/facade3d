function ProjectImagesOntoPointCloud()

    subset = 'train';
    nColors = 3;
    nLabels = 8;
    
    dirName = '/esat/sadr/amartino/Facade3D/data/';
    imageLocation = '/esat/sadr/amartino/Facade3D/data/images/';
    labelLocation = '/esat/sadr/amartino/Facade3D/data/labels/';
    
    % Read the dense point cloud from CMVS
    % Low-res mesh
    [points,normals,~]=ReadPCLFromPly([dirName 'pcl.ply']);
    
    % High-res mesh
%     [points,normals,~,~,~]=ReadMeshFromPly([dirName '0_fullResMesh/mesh_normals.ply']);
    
    % Read camera information from CMVS
    cameras = ImportCameras([dirName 'cameras.txt']);

    
    % LOAD DATA FILE NAMES & INDEX
    filename = [dirName 'listtrain.txt'];
    fid = fopen(filename);
    if fid==-1
        fatal();
    end
    file_str_idx = textscan(fid, '%s'); fclose(fid);
    nImages = length(file_str_idx{1});
    
    file_str_idx = file_str_idx{1};
    
   
    
%     nImages = length(file_str_idx);

    % Mapping between cmvs images and labeled image
    colorsPerPoint = zeros(length(points),nImages,nColors);
    labelsPerPoint = zeros(length(points),nImages,nLabels);
    
    % For each image
    fprintf('Projecting %d images onto the point cloud\n',nImages);
    cnt = 0;
    for i=1:nImages
        basename = file_str_idx{i};
        bb=cellfun(@(x)strcmp(x.originalImageFilename(1:end-4),basename),cameras,'UniformOutput',false);
        camIdx = find(cell2mat(bb));
        
        if mod(i,10)==0
            fprintf('\n%d   ',i);
        end
        if isempty(camIdx)
            error('No camera found!');
        end
        
        height = cameras{camIdx}.principalPoint(2)*2;
        width = cameras{camIdx}.principalPoint(1)*2;
        
        imageFilename = [imageLocation basename '.jpg'];
        labelingFilename = [labelLocation basename '.png'];

        if exist(imageFilename,'file')
            cnt = cnt+1;

            origImg = imread(imageFilename);
          
            if height==size(origImg,2) && width==size(origImg,1)
                origImg = imrotate(origImg,90);
            else
                error('Camera-image size mismatch!');
            end

            % Backproject the colors
            for c=1:nColors
                colorsPerPoint(:,i,c) = BackProjectLabeling(points,origImg(:,:,c),cameras{camIdx});
            end

            fprintf('o');
        else
            fprintf('x');
            continue;
        end
        
        
        if exist(imageFilename,'file')
            cnt = cnt+1;

            labeling = imread(labelingFilename);
          
            if height==size(labeling,2) && width==size(labeling,1)
                labeling = imrotate(labeling,90);
            else
                error('Camera-image size mismatch!');
            end
            
            labeling = Image2Label(labeling);

            % Backproject the colors
            for c=1:nLabels
                labelsPerPoint(:,i,c) = BackProjectLabeling(points,(labeling==c),cameras{camIdx});
            end

            fprintf('o');
        else
            fprintf('x');
            continue;
        end
        

    end
    
    colors = [squeeze(sum(colorsPerPoint,2)./sum(colorsPerPoint~=0,2))]';
    colors(isnan(colors)) = 0;
    colors = round(colors');
   
    fprintf('Projected %d images onto the %s point cloud.\n',cnt, subset);

    ExportMesh([dirName 'pcl_colored.ply'],points,normals,colors,[],[]);
  
end