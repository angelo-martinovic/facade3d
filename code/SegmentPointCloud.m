function newColors = SegmentPointCloud
    addpath /users/visics/amartino/RNN_link/RNN/repo/source/GCMex2.3/
    addpath /users/visics/amartino/RNN_link/RNN/repo/source/
    
    cmvsLocation = '/esat/sadr/amartino/monge3dRight/reconstruction.nvm.cmvs/00/';
    
    % Load point cloud
    [points,normals,colors]=ReadPointCloudFromPly([cmvsLocation 'models/option-0000.ply']);
    
    n = length(points);
    
    %% Label cost
    C = 8;
    labelcost = ones(C,C);
    labelcost(1:C+1:end)=0;
    
    %% Initial labels
    segclass = zeros(1,n);
     
    %% Unary potentials
    % Read camera information from CMVS
    cameras = ImportCameras([cmvsLocation 'cameras_v2.txt']);

   
    unary = zeros(length(points),C);
    unaryDet = zeros(length(points),C);
    
    if ~exist('unary.mat','file')
        % For each camera
        for i=1:length(cameras)
            fprintf('.');

            height = cameras{i}.principalPoint(2)*2;
            width = cameras{i}.principalPoint(1)*2;
            
            % Read the image predictions
            basename = cameras{i}.visualizeImageName(1:end-4);
            %basename = cameras{i}.originalImageFilename(1:end-4);
            
%             homography=dlmread([cmvsLocation 'visualizeRect/' basename '.rect.dat']);
%             
%             target = imread([cmvsLocation 'visualize/' basename '.jpg']);
 
            
%             load([cmvsLocation 'visualizeRect/' basename '.label_layer1.mat']);
            load([cmvsLocation 'visualizeRect/' basename '.label_layer2_withDet.mat']);
            segMap = zeros(height,width,8);
            for c=1:8
                segMap(:,:,c) = segMap(:,:,c) + 0.93*(outImg==c);
                segMap(:,:,c) = segMap(:,:,c) + 0.01*(outImg~=c);
            end
            % Should have loaded this variable
            if ~exist('segMap','var')
                error('Probability map not found.')
            end

           
            detMap = detectionMaps(1).detectionMap;

%             segMap2 = zeros(height,width,size(segMap,3));
%             for c=1:size(segMap,3)
%                 tmp = segMap(:,:,c);
%                 tmp2 = rewarp(target,tmp,homography);
% %                 tmp3 = imrotate(tmp2,90);
%                 segMap2(:,:,c) = imresize(tmp2,[height width],'nearest');
%             end
%             segMap = segMap2;

             % The cmvs pictures are resized and (sometimes) rotated
    %         [~,outImg] = max(segMap2,[],3);
    %         figure(2);imagesc(outImg);
    %         outImg = imrotate(outImg,90);
    %         outImg = imresize(outImg,[height width],'nearest');

            pointEnergies = ProjectProbabilitiesToPointCloud(points,segMap,cameras{i});
            
            pointEnergiesDet = ProjectProbabilitiesToPointCloud(points,detMap,cameras{i});

            unary = unary + pointEnergies;
            unaryDet = unaryDet + pointEnergiesDet;
            %newLabels_i = BackProjectLabeling(points,outImg,cameras{i});
            %newLabels(:,i) = newLabels_i;

        end
        save('unary.mat','unary','unaryDet');
    else
        load('unary.mat');
    end
    
    alpha = 1.65;
    unary = unary + alpha * unaryDet;
    
    visualize = 1;
    if visualize
        [~,newLabels] = min(unary,[],2);

        % Haussmann colormap
        colormap = [255 0 0;
                       255  255 0;
                       128  0   255;
                       255  128 0;
                       0    0   255;
                       128  255 255;
                       0    255 0;
                       0    0   255];
        colormap = colormap / 256;

        % Mapping colors
        newColors = colormap(newLabels,:);

        ExportPointCloud('models/det3d_det_rect_out.ply',points,normals,round(255*newColors));

    end
    
   
    %% Pairwise potentials
    % Find k nearest neighbors
    k = 4;
    [idx,d]=knnsearch(points,points,'k',k+1);
    
    % Remove 1st column, corresponding to same element
    idx = idx(:,2:end);
    d = d(:,2:end);
    
    % Create sparse adjacency matrix
    % i = [11112222...nnnn]
    ii = zeros(1,k*n);
    for i=1:k
        ii(i:k:end)=1:n;
    end
    
    jj = reshape(idx',1,k*n);
    ss = reshape(d',1,k*n);
    
    % Pairwise costs: w = exp(-(d/sigma)^2)
    sigma = mean(ss(:));
    ss = 3* exp( - (ss/sigma).^2  );    % When ss==mean(ss), penalty should be 1
    
    pairwise = sparse(ii,jj,ss,n,n);
    % There will typically be many connected components
    % TODO: connect all of them
    %[comps,sizes] = components(pairwise);
    
    
    %% Calling the MEX file
    unary = unary';
    disp('Running the graph cut...');
    lambda = 1.52;
    labelcost = lambda * labelcost;
    [labels,E,Eafter] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    disp('Done.');
    labels = labels + 1;

    E_begin = E;
    E_end = Eafter;
    
    % Haussmann colormap
    colormap = [255 0 0;
                   255  255 0;
                   128  0   255;
                   255  128 0;
                   0    0   255;
                   128  255 255;
                   0    255 0;
                   0    0   255];
    colormap = colormap / 256;
       
    % Mapping colors
    newColors_crf = colormap(labels,:);
    
%     disp(sum(newColors==newColors_crf));
    
    disp('Exporting to ply...');
    ExportPointCloud('models/det3d_det_rect_out_crf.ply',points,normals,round(255*newColors_crf));

    return;
    
    %result = reshape(labels,W,H)';
   
    
end