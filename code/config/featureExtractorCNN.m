classdef featureExtractorCNN < featureExtractor2D
    %featureExtractorCNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        netName
        useGpu
        batchSize
        targetLayers
        layerOutputsIdx
    end
    
    methods
        function F = featureExtractorCNN(workFolder)
           
            F@featureExtractor2D(workFolder);
            F.name = 'CNN';
             
            run([vl_rootnn() '/matlab/vl_setupnn']);
            
            % Conv1 conv2 conv3 conv4 conv5
            F.nFeats = 64+128+256+512+512; 
            
            % Scale features to norm of 1?
            F.scaleFeats = false;
            
            %     obj.modelName = 'imagenet-caffe-ref.mat';
            %     obj.modelName = 'imagenet-vgg-f.mat';
            F.netName = 'imagenet-vgg-verydeep-16.mat';
            F.targetLayers = {'conv1_2', 'conv2_2', 'conv3_3', 'conv4_3', 'conv5_3'}; 
           
            F.useGpu = true; 
            F.batchSize = 1; 
            
        end
        
        function ExtractFeatures(obj)
            dl = DispatchingLogger.getInstance();
            
            % Load the CNN
            net = load(obj.netName);

            layerNames = cellfun(@(x) x.name,net.layers,'UniformOutput',false);
            targetLayersIdx = cellfun(@(x)find(strcmp(layerNames,x)),obj.targetLayers,'UniformOutput',false);
            targetLayersIdx = cat(2,targetLayersIdx{:});
            obj.layerOutputsIdx = targetLayersIdx+1;
            
            % Get the image names from the directory
            imageList = dir([obj.workFolder '*.jpg']);

            N = length(imageList);

            dl.Log(VerbosityLevel.Debug,sprintf(' - CNN features from %s.\n',strjoin(obj.targetLayers,',')));
            dl.Log(VerbosityLevel.Debug,sprintf(' - Extracting features from %d images.\n',N));
    
            % Extract features for every image
            for i=1:N
                
                imageName = [obj.workFolder imageList(i).name(1:end-4) '.jpg'];
                segName = [obj.workFolder imageList(i).name(1:end-4) '.seg'];
                featName = [obj.workFolder imageList(i).name(1:end-4) '.features.' obj.name '.mat'];
                
                dl.Log(VerbosityLevel.Debug,sprintf(' - Extracting features from image %d...\n',i));
                if ~exist(featName,'file')
                    % Load image
                    im = imread(imageName);

                    % Load segmentation
                    seg = dlmread(segName);

                    % Extract features
                    features = extract_segment_features_rcnn_oneImage(obj,im,seg,net); 

                    if obj.scaleFeats
                        features = bsxfun(@rdivide,features,sqrt(sum(features.^2,2))); %#ok<NASGU>
                    end
                    
                    % Save features
                    save(featName,'features');
                end
            end
        end
    end
    
    methods(Access=private)
        function features = extract_segment_features_rcnn_oneImage(obj,im,seg,net)
            dl = DispatchingLogger.getInstance();
             
            % Get the superpixel segmentation
            segIDs = unique(seg(:));
            nSuperpixels = numel(segIDs);
            meanImage = net.normalization.averageImage;

            % Consider resizing the image if it doesnt fit into memory
            %  im = imresize(im,0.5);

            imSize = size(im);
            imSize = imSize(1:2);

            % Subtract the mean image?
            % im_ = im;
            im_ = double(im)-imresize(meanImage,imSize);
            im_ = single(im_);
            
            try
                % GPU version
                if obj.useGpu
                    net = vl_simplenn_move(net, 'gpu');
                    im_ = gpuArray(im_);
                end

                % Calculate the response of the network in the convolutional
                % mode
                dl.Log(VerbosityLevel.Debug,sprintf(' - - Calling CNN...\n'));

                res = vl_simplenn(net, im_, [], [], 'keepLayerOutput',obj.layerOutputsIdx);
            catch err
                dl.Log(VerbosityLevel.Warning,sprintf([' - - Error running on GPU: %s.'...
                    'Disabling GPU and attempting to run on CPU...\n'],err.message));
                
                obj.useGpu = false;
                net = vl_simplenn_move(net, 'cpu');
                im_ = gather(im_);
                
                res = vl_simplenn(net, im_, [], [], 'keepLayerOutput',obj.layerOutputsIdx);
            end
            dl.Log(VerbosityLevel.Debug,sprintf(' - - CNN done.\n'));
            dl.Log(VerbosityLevel.Debug,sprintf(' - - Gathering features...\n'));

            clear im im_ meanImage
            L = length(obj.layerOutputsIdx);

            filterSizes = cellfun(@(x)size(x.filters),net.layers(obj.layerOutputsIdx-1),'UniformOutput',false);
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
                tmp = squeeze(gather(res(obj.layerOutputsIdx(i)).x));
                %     ShowActivations(uint8(im),tmp(:,:,1:25),5,5,cnn.layerOutputsIdx{i}-1);
                % The result is a H*W matrix, we need to upsample it to imSize
                feats(:,:,filterSizeRanges(i)+1:filterSizeRanges(i+1)) = imresize(tmp,imSize,'bilinear');
            end
            dl.Log(VerbosityLevel.Debug,sprintf(' - - Done.\n'));
            % Pool the features in each superpixel

            dl.Log(VerbosityLevel.Debug,sprintf(' - - Pooling features over superpixels...\n'));
            feats = reshape(feats,[imSize(1)*imSize(2) nFeats]);
            segVec = reshape(seg,[imSize(1)*imSize(2) 1])+1;

            features = zeros(nSuperpixels,nFeats); 
            for i=1:nFeats
                features(:,i) = accumarray(segVec,feats(:,i));
            end
            segSizes = hist(segVec,1:max(segVec));

            % Average pooling
            features = bsxfun(@rdivide,features,segSizes');
            dl.Log(VerbosityLevel.Debug,sprintf(' - - Done.\n'));
        end
    end
    
end

