% Runs the first two layers of ATLAS.
% Collects the calculated marginals and the detector output on the test set.
function LabelImagesATLAS(obj)
    dl = DispatchingLogger.getInstance();
    
    dl.Log(VerbosityLevel.Info,sprintf('Starting 2D image labeling (ATLAS)...\n'));
    
    % Image names
    imageNames = obj.LoadFilenames('eval');
        
    % Label each image
    tic;
    nImages = length(imageNames);
    
    disp('-----------------');
    cf = obj.config;
    cacheFoundCount = 0;
    retCodes = zeros(1,nImages);
    msgs = cell(1,nImages);
    pb = ProgressBar(nImages);
    parfor (i = 1:nImages,obj.config.nWorkers)
%     for i = 1:nImages
        outputFilename = get_adr('image_classifier_unaries',cf,cf.c2D.classifier.name,imageNames{i});
        
        if exist(outputFilename,'file') && cf.useCache
            cacheFoundCount = cacheFoundCount + 1;
            pb.progress();
            continue;
        else
            [retCodes(i),msgs{i}] = LabelOneImage(obj, imageNames{i});
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

function [retCode,msg] = LabelOneImage(obj,imageName)
    retCode = 0;
    msg = [];
%     dl = DispatchingLogger.getInstance();
    
    imageFilename = get_adr('2D_image',obj.config,imageName);
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
    imOrig = imread(get_adr('2D_image_orig',obj.config,imageName));
    
    % Non-rectified image size
    heightOrig = size(imOrig,1);
    widthOrig = size(imOrig,2);
    
    %% First layer
    segmentation = dlmread(get_adr('2D_segmentation',obj.config,imageName));
    % Since segments start from 0
    segmentation = segmentation + 1;   

    % Number of segments in the image
    numSegs = max(segmentation(:));
    
    % Classifier output
    prob_estimates = dlmread(get_adr('2D_marginals',obj.config,obj.config.c2D.classifier.name,imageName));

    if length(prob_estimates)~=numSegs
        retCode = -1;
        msg = sprintf(' - Image %s: segment count mismatch!',imageName);
        return;
    end

    % Label probability maps   
    nClasses = size(prob_estimates,2) - 1;
    if nClasses~=obj.config.nClasses
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
    nDetectors = length(obj.config.c2D.detectors);
    
    detectionData = cell(1,nDetectors);
    for i=1:nDetectors
        detector = obj.config.c2D.detectors{i};
        detectionFilename = get_adr('2D_detectionsEval', obj.config, detector.name, imageName);
        detectionData{i} = detector.ProbabilityMapFromDetections(detectionFilename,nClasses,height,width);
    end

    % Unrectification
    if obj.config.rectificationNeeded
        scale = dlmread(get_adr('2D_scale',obj.config,imageName));
        homography = dlmread(get_adr('2D_rectParams',obj.config,imageName));

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
    [outImg,~,~] = obj.RunCRF(segMap,detectionData);
    
    % Output
    outputDir = get_adr('2D_classification',obj.config,obj.config.c2D.classifier.name);
    
    % Save overlayed result image to disk
    mkdirIfNotExist([outputDir '/layer1/']);
    mkdirIfNotExist([outputDir '/layer2/']);
        
    alpha = 0.5;
    imwrite(imlincomb(alpha, Label2Image(oldImg,obj.config.cm), 1-alpha, image),...
        [outputDir '/layer1/' imageName '_overlay.png']);
    imwrite(imlincomb(alpha, Label2Image(outImg,obj.config.cm), 1-alpha, image),...
        [outputDir '/layer2/' imageName '_overlay.png']);
    
    imwrite(Label2Image(oldImg,obj.config.cm),[outputDir '/layer1/' imageName '.png']);
    imwrite(Label2Image(outImg,obj.config.cm),[outputDir '/layer2/' imageName '.png']);
    
    for i=1:nDetectors
        dmap = detectionData{i}(:,:,obj.config.c2D.detectors{i}.class);
        maxVal = max(dmap(:));
        minVal = min(dmap(:));

        detMap = uint8((dmap-minVal) / (maxVal-minVal) * 255);
        detMap = repmat(detMap,[1 1 3]);
    
        imwrite(imlincomb(alpha, detMap, 1-alpha, image),...
        [outputDir '/layer2/' imageName 'detections_' obj.config.c2D.detectors{i}.name '.png']);
    end
    
    % A big mat file
    segPotentials = segMap; layer1_labeling = oldImg; layer2_labeling = outImg; %#ok<NASGU>
    save(get_adr('image_classifier_unaries',obj.config,obj.config.c2D.classifier.name,imageName),'segPotentials','detectionData','layer1_labeling','layer2_labeling');

end
