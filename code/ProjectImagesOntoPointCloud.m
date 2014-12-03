% function ProjectImagesOntoPointCloud()

    subset = 'all';
    nClasses = 3;
    
    dirName = '/esat/sadr/amartino/monge428New/data/';
    imageLocation = '/esat/sadr/amartino/monge428New/data/image/';
    
    % Read the dense point cloud from CMVS
    % Low-res mesh
%     [points,~,~]=ReadPCLFromPly([dirName 'pcloud_gt_train_new_GCO.ply']);
    
    % High-res mesh
    [points,normals,~,faces,~]=ReadMeshFromPly([dirName '0_fullResMesh/mesh_normals.ply']);
    
    % Read camera information from CMVS
    cameras = ImportCameras([dirName 'cameras.txt']);

    % Subset of images used to project the labeling
    file_str_idx = LoadFilenames(dirName,subset);
    nImages = length(file_str_idx);

    % Mapping between cmvs images and labeled image
    colorsPerPoint = zeros(length(points),nImages,nClasses);
        
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
        
        labelingFilename = [imageLocation basename '.jpg'];

        if exist(labelingFilename,'file')
            cnt = cnt+1;

            origImg = imread(labelingFilename);
          
            if height==size(origImg,2) && width==size(origImg,1)
                origImg = imrotate(origImg,90);
            else
                error('Camera-image size mismatch!');
            end

            % Backproject the colors
            for c=1:nClasses
                colorsPerPoint(:,i,c) = BackProjectLabeling(points,origImg(:,:,c),cameras{camIdx});
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

    ExportMesh([dirName 'fullRes_mesh_colors_normals.ply'],points,normals,colors,faces,[]);
  
% end