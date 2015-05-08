% Runs the first two layers of ATLAS.
% Collects the calculated marginals and the detector output on the test set.
function LabelImagesATLAS()
    dl = DispatchingLogger.getInstance();
    
    dl.Log(VerbosityLevel.Info,sprintf('Starting 2D image labeling (ATLAS)...\n'));
    
    % Image names
    imageNames = LoadFilenames('eval');
        
    % Label each image
    tic;
    nImages = length(imageNames);
    
    disp('-----------------');
    cf = DatasetConfig.getInstance();
    datasetName = cf.name;
    cacheFoundCount = 0;
    retCodes = zeros(1,nImages);
    msgs = cell(1,nImages);
    pb = ProgressBar(nImages);
    parfor (i = 1:nImages,cf.nWorkers)
%     for i = 1:nImages
        cfLocal = DatasetConfig.getInstance(datasetName);
        outputFilename = get_adr('image_classifier_unaries',cfLocal.c2D.classifier.name,imageNames{i});
        
        if exist(outputFilename,'file') && cfLocal.useCache
            cacheFoundCount = cacheFoundCount + 1;
            pb.progress();
            continue;
        else
            [retCodes(i),msgs{i}] = LabelOneImage(cfLocal,imageNames{i});
        end
        pb.progress();
    end
    pb.stop();
    
    atlasTime = toc;
    failCount = sum(retCodes==-1);
    if failCount~=0
       dl.Log(VerbosityLevel.Error,sprintf('ATLAS finished. %d images failed.\n',failCount));
       dl.Log(VerbosityLevel.Error,sprintf('Fail messages: %s\n',msgs{retCodes==-1}));
       fatal();
    end
    
    warningCount = sum(retCodes==1);
    if warningCount~=0
       dl.Log(VerbosityLevel.Warning,sprintf('ATLAS finished. %d images with warnings.\n',warningCount));
       dl.Log(VerbosityLevel.Warning,sprintf('Warning messages: %s\n',msgs{retCodes==1}));
    end
    
    dl.Log(VerbosityLevel.Info,...
        sprintf('ATLAS finished. %d results loaded from cache. Elapsed time: %.2f seconds.\n',cacheFoundCount,atlasTime));   
end

function [retCode,msg] = LabelOneImage(cf,imageName)
    retCode = 0;
    msg = [];
%     dl = DispatchingLogger.getInstance();
    
    imageFilename = get_adr('2D_image',imageName);
    if ~exist(imageFilename,'file')
        retCode = 1;
        msg = sprintf(' - Image %s does not exist. Skipping...',imageFilename);
        return;
    end
    
    % Rectified image
    image = imread(imageFilename);
    
    height = size(image,1);
    width  = size(image,2);
    % Original image - before rectification
    imOrig = imread(get_adr('2D_image_orig',imageName));
    
    % Non-rectified image size
    heightOrig = size(imOrig,1);
    widthOrig = size(imOrig,2);
    
    %% First layer
    segmentation = dlmread(get_adr('2D_segmentation',imageName));
    % Since segments start from 0
    segmentation = segmentation + 1;   

    % Number of segments in the image
    numSegs = max(segmentation(:));
    
    % Classifier output
    prob_estimates = dlmread(get_adr('2D_marginals',cf.c2D.classifier.name,imageName));

    if length(prob_estimates)~=numSegs
        retCode = -1;
        msg = sprintf(' - Image %s: segment count mismatch!',imageName);
        return;
    end

    % Label probability maps   
    nClasses = size(prob_estimates,2) - 1;
    if nClasses~=cf.nClasses
        retCode = -1;
        msg = sprintf('Mismatch in number of classes between the configuration and classifier output!');
        return;
    end
    segMap = zeros(size(segmentation,1),size(segmentation,2),nClasses);
    % Create a probability map for each pixel
    for c=1:nClasses
        cVec = prob_estimates(:,c+1);
        segMap(:,:,c) = cVec(segmentation);
    end
    
    % Superpixel output
    [~,oldImg] = max(segMap,[],3);
    
    %% 2D Second layer 
    % Detectors
    nDetectors = length(cf.c2D.detectors);
    
    detectionData = cell(1,nDetectors);
    for i=1:nDetectors
        detector = cf.c2D.detectors{i};
        detectionFilename = get_adr('2D_detectionsEval', detector.name, imageName);
        detectionData{i} = detector.ProbabilityMapFromDetections(detectionFilename,nClasses,height,width);
    end

    % Unrectification
    if cf.rectificationNeeded
        scale = dlmread(get_adr('2D_scale',imageName));
        homography = dlmread(get_adr('2D_rectParams',imageName));

        resizedImage = imresize(image,scale);

        imageUnrect = zeros(heightOrig,widthOrig,3);
        for i=1:3
            imageUnrect(:,:,i) = rewarp(imOrig,resizedImage(:,:,i),homography);
        end
        image = uint8(imageUnrect);
        
        
        detectionDataUnrect = detectionData;
        for i=1:nDetectors
            detectionDataUnrect{i} = zeros(heightOrig,widthOrig,nClasses);
        end

        segMapUnrect = zeros(heightOrig,widthOrig,nClasses);
        for i=1:nClasses
            s2 = imresize(segMap(:,:,i),scale,'nearest');
            segMapUnrect(:,:,i)=rewarp(imageUnrect(:,:,1),s2,homography,1/nClasses);
            for j=1:nDetectors
                d2 = imresize(detectionData{j}(:,:,i),scale,'nearest'); 
                detectionDataUnrect{j}(:,:,i) = rewarp(imageUnrect(:,:,1),d2,homography,1/nClasses);
            end
        end
        segMap = segMapUnrect;
        detectionData = detectionDataUnrect;

        clear imageUnrect segMapUnrect detectionDataUnrect;

        oldImg = imresize(oldImg,scale,'nearest');
        oldImg = rewarp(imOrig,oldImg,homography);
    end
    
    % Run CRF
    [outImg,~,~] = RunCRF(segMap,detectionData);
    
    % Output
    outputDir = get_adr('2D_classification',cf.c2D.classifier.name);
    
    % Save overlayed result image to disk
    mkdirIfNotExist([outputDir '/layer1/']);
    mkdirIfNotExist([outputDir '/layer2/']);
        
    alpha = 0.5;
    imwrite(imlincomb(alpha, Label2Image(oldImg,cf.cm), 1-alpha, image),...
        [outputDir '/layer1/' imageName '_overlay.png']);
    imwrite(imlincomb(alpha, Label2Image(outImg,cf.cm), 1-alpha, image),...
        [outputDir '/layer2/' imageName '_overlay.png']);
    
    imwrite(Label2Image(oldImg,cf.cm),[outputDir '/layer1/' imageName '.png']);
    imwrite(Label2Image(outImg,cf.cm),[outputDir '/layer2/' imageName '.png']);
    
    for i=1:nDetectors
        dmap = detectionData{i}(:,:,cf.c2D.detectors{i}.class);
        maxVal = max(dmap(:));
        minVal = min(dmap(:));

        detMap = uint8((dmap-minVal) / (maxVal-minVal) * 255);
        detMap = repmat(detMap,[1 1 3]);
    
        imwrite(imlincomb(alpha, detMap, 1-alpha, image),...
        [outputDir '/layer2/' imageName 'detections_' cf.c2D.detectors{i}.name '.png']);
    end
    
    % A big mat file
    segPotentials = segMap; layer1_labeling = oldImg; layer2_labeling = outImg; %#ok<NASGU>
    save(get_adr('image_classifier_unaries',cf.c2D.classifier.name,imageName),'segPotentials','detectionData','layer1_labeling','layer2_labeling');

end

function [result,E_begin,E_end] = RunCRF(unarySegmentationPotentials,unaryDetectionPotentials)  
    cf = DatasetConfig.getInstance();
     
    crf = cf.c2D.crf;
    nClasses = cf.nClasses;
    
    %% Image size
    H = size(unarySegmentationPotentials,1);
    W = size(unarySegmentationPotentials,2);
       
    %% Initial labels
    [~,oldImg] = max(unarySegmentationPotentials,[],3);
    oldImg = oldImg-1;
    segclass = reshape(oldImg',W*H,1);

    %% Pairwise term
    % 4 - neighborhood: 1-ball with L1 distance
    [ii, jj] = sparse_adj_matrix([W H], 1, 1);
    pairwise = sparse(ii,jj,ones(1,numel(ii)), W * H, W * H);
    pairwise(1:size(pairwise,1)+1:end)=0;
                                          
    %% Unary term
    unary = zeros(nClasses,W*H);
    nDetectors = length(unaryDetectionPotentials);
    unary_mat = crf.weightUnarySegmentation * (- log(unarySegmentationPotentials) );
        
    for d=1:nDetectors
        unary_mat = unary_mat + crf.weightsUnaryDetectors(d) * (-log(unaryDetectionPotentials{d}));
    end
    
    for c=1:nClasses
        unary(c,:) = reshape(unary_mat(:,:,c)',1,W*H);
    end
           
    %% Label term
    labelcost = crf.labelCost;

    % Final expression
    pairwise = crf.weightPairwise*pairwise;

	%% Calling the MEX file
    [labels,E,Eafter] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    labels = labels + 1;

    E_begin = E;
    E_end = Eafter;
    
    result = reshape(labels,W,H)';
end
