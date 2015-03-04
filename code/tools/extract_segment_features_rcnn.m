function features=extract_segment_features_rcnn(workFolder)
    % setup MtConvNet in MATLAB
    addpath external/matconvnet/matlab
    run([vl_rootnn() '/matlab/vl_setupnn']);

%     cnn_options.modelName = 'imagenet-caffe-ref.mat';
%     cnn_options.modelName = 'imagenet-vgg-f.mat';
    cnn_options.modelName = 'imagenet-vgg-verydeep-16.mat';
    cnn_options.useGpu = true; 
    cnn_options.batchSize = 1; 
    cnn_options.verbose = true;
        
    net = load([vl_rootnn() '/models/' cnn_options.modelName]);
    
%     targetLayers = {'conv3'};
%     targetLayers = {'conv3', 'conv4', 'conv5'};
    targetLayers = {'conv1_2', 'conv2_2', 'conv3_3', 'conv4_3', 'conv5_3'};
%     targetLayers = {'conv3_3', 'conv4_3'};
    layerNames = cellfun(@(x) x.name,net.layers,'UniformOutput',false);
    targetLayersIdx = cellfun(@(x)find(strcmp(layerNames,x)),targetLayers,'UniformOutput',false);
    targetLayersIdx = cat(2,targetLayersIdx{:});
    
    cnn_options.layerOutputsIdx = targetLayersIdx+1;
    
    
    imageList = dir([workFolder '*.jpg']);

    N = length(imageList);
    
    if cnn_options.verbose
        fprintf('Extracting features from %d images.\n',N);
        fprintf('Extracting CNN features from %s.\n',targetLayers{:});
    end
    
    pb = ProgressBar(N);
    for i=1:N
        
        imageName = [workFolder imageList(i).name];
        segName = [workFolder imageList(i).name(1:end-4) '.seg'];
        featName = [workFolder imageList(i).name(1:end-4) '.featuresCNN.mat'];

        % Load image
        im = imread(imageName);

        % Load segmentation
        seg = dlmread(segName);
            
        % Extract features
        features = extract_segment_features_rcnn_oneImage2(im,seg,net,cnn_options);

        % Save features
        save(featName,'features');
        pb.progress;
    end
    pb.stop;
end
    

function features = extract_segment_features_rcnn_oneImage2(im,seg,net,cnn_options)
    useGpu = cnn_options.useGpu;
    
    segIDs = unique(seg(:));
    nSuperpixels = numel(segIDs);
    meanImage = net.normalization.averageImage;

%     im = imresize(im,0.5);
    
    imSize = size(im);
    imSize = imSize(1:2);
    
    % Subtract the mean image?
%     im_ = im;
    im_ = double(im)-imresize(meanImage,imSize);
    
    if useGpu
        net = vl_simplenn_move(net, 'gpu');
        im_ = gpuArray(single(im_));
    end

    % Calculate the response of the network on the full image 
    fprintf('Calling CNN...');
    res = vl_simplenn(net, im_, [], [], 'keepLayerOutput',cnn_options.layerOutputsIdx);
    fprintf('CNN done.\n');
    
    fprintf('Gathering features...');
    clear im im_ meanImage
    L = length(cnn_options.layerOutputsIdx);
    
    filterSizes = cellfun(@(x)size(x.filters),net.layers(cnn_options.layerOutputsIdx-1),'UniformOutput',false);
    filterSizes = cat(1,filterSizes{:});
    filterSizes = filterSizes(:,3);
    filterSizeRanges = [0;cumsum(filterSizes)];
    
    imSizeOrig = imSize;
    subsampling = 1;
    imSize = imSizeOrig/subsampling;
    seg = imresize(seg,imSize,'nearest');
    
    nFeats = sum(filterSizes);
    feats = zeros(imSize(1),imSize(2),nFeats,'single');
    for i=1:L
        tmp = squeeze(gather(res(cnn_options.layerOutputsIdx(i)).x));
        %     ShowActivations(uint8(im),tmp(:,:,1:25),5,5,cnn.layerOutputsIdx{i}-1);
        % The result is a H*W matrix, we need to upsample it to imSize
        feats(:,:,filterSizeRanges(i)+1:filterSizeRanges(i+1)) = imresize(tmp,imSize,'bilinear');
    end
    fprintf('Done.\n');
    % Pool the features in each superpixel
    
    fprintf('Pooling features...');
    feats = reshape(feats,[imSize(1)*imSize(2) nFeats]);
    segVec = reshape(seg,[imSize(1)*imSize(2) 1])+1;
   
    features = zeros(nSuperpixels,nFeats); 
    for i=1:nFeats
        features(:,i) = accumarray(segVec,feats(:,i));
    end
    segSizes = hist(segVec,1:max(segVec));
    
    % Average pooling
    features = bsxfun(@rdivide,features,segSizes');
    fprintf('Done.\n');
end

function ShowActivations(im,featureActivation,nRows,nCols,layerName)

    figure;
    title('Activations');
        
    for i=1:nRows
        for j=1:nCols
            p = (i-1)*nRows+j;
            if p>size(featureActivation,3)
                return;
            end
            subplot(nRows,nCols,p);
            imshow( im ); hold on;
            h = imagesc(featureActivation(:,:,p));
            set (h, 'AlphaData', .8);
            set(gca,'xtick',[],'ytick',[],'layer','bottom','box','on')
            xlabel(num2str(p));
        end
    end
    suptitle(layerName);
end

function features = extract_segment_features_rcnn_oneImage(im,seg,net,cnn_options)
    useGpu = cnn_options.useGpu;
    batchSize = cnn_options.batchSize;
    
    segIDs = unique(seg(:));
    N = numel(segIDs);
    
    if cnn_options.verbose
        fprintf('Extracting features from %d segments.\n',N);
    end
    imSize = net.normalization.imageSize(1:2);
    imDepth = net.normalization.imageSize(3);
    avgImg = net.normalization.averageImage;
    
    layerNames = cellfun(@(x) x.name,net.layers,'UniformOutput',false);
    targetLayerIdx = find(strcmp(layerNames,cnn_options.outputLayerName),1);
    featureSize = size(net.layers{targetLayerIdx}.biases,2);
    
    layerOutputIdx = targetLayerIdx+1;
    
    if useGpu
        net = vl_simplenn_move(net, 'gpu');
    end
    
    features = zeros(N,2*featureSize);
    
    endPointer = 0;
    batchID = 0;
    while(endPointer<N)
        
        startPointer = endPointer+1;
        endPointer = min([startPointer+batchSize-1 N]);
        
        batchID = batchID+1;
        curBatchSize = endPointer-startPointer+1;
        
        if cnn_options.verbose
            fprintf('Batch %d, segments %d to %d (%d total)...\n',batchID,startPointer,endPointer,N);
        end
        
        if useGpu
            bboxesFull = zeros([imSize imDepth curBatchSize],'gpuArray');
            bboxesFG = zeros([imSize imDepth curBatchSize],'gpuArray');
        else
            bboxesFull = zeros([imSize imDepth curBatchSize]);
            bboxesFG = zeros([imSize imDepth curBatchSize]);
        end

        % For each segment
        for i=startPointer:endPointer
            % Get the segmentation box
            mask = (seg==segIDs(i));
            [ii,jj] = find(mask);
            bbox = im(min(ii):max(ii),min(jj):max(jj),:);
            fgMask = mask(min(ii):max(ii),min(jj):max(jj),:);

            % Warp the box to net image size
            bboxesFull(:,:,:,i-startPointer+1) = single(imresize(bbox,imSize))-avgImg;

            % Mask out the background
            maskW = single(repmat(imresize(fgMask,imSize),[1 1 imDepth]));
            bboxesFG(:,:,:,i-startPointer+1) = maskW.*single(imresize(bbox,imSize)) + (1-maskW).* avgImg - avgImg;

        end

        % Extract features (fc6) from the warped regions
        featsFull = ExtractFeatures(single(bboxesFull),net,layerOutputIdx);

        % Extract features (fc6) from the masked regions
        featsFG = ExtractFeatures(single(bboxesFG),net,layerOutputIdx);

        % Add features to the map
        features(startPointer:endPointer,:) = [featsFull' featsFG'];
          
    end
        
end

function feats = ExtractFeatures(im,net,layerOutputIdx)

    res = vl_simplenn(net,im, [], [], 'keepLayerOutput',layerOutputIdx);
    
    feats = squeeze(gather(res(layerOutputIdx).x));
end
