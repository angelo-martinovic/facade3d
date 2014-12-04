% Runs the first two layers of ATLAS.
% Collects the calculated marginals and the detector output on the test set.
function LabelImagesATLAS(obj)

    % Image names
    imageNames = obj.LoadFilenames('eval');
    
    % Generate data filenames
    % Provided by user
    origImageFilenames = strcat(get_adr('2D_images',obj.config),strcat(imageNames,'.jpg'));
    groundTruthFilenames = strcat(get_adr('2D_labels',obj.config),strcat(imageNames,'.png'));
    
    % Generated by PrepareData
    segFilenames = strcat(get_adr('work',obj.config),strcat(imageNames,'.seg'));
    imageFilenames = strcat(get_adr('work',obj.config),strcat(imageNames,'.jpg'));
    scaleFilenames = strcat(get_adr('work',obj.config),strcat(imageNames,'_scale.dat'));
    rectFilenames = strcat(get_adr('work',obj.config),strcat(imageNames,'_rect.dat'));
    
    % Generated by ClassifyWithSVM
    classificationFilenames = strcat(get_adr('2D_classification',obj.config,obj.classifierName),strcat(imageNames,'.marginal.txt'));

    files = struct('segFilename',segFilenames,'imageFilename',imageFilenames,...
        'scaleFilename',scaleFilenames,'rectFilename',rectFilenames,...
        'groundTruthFilename',groundTruthFilenames,'origImageFilename',origImageFilenames,...
        'classificationFilename',classificationFilenames);
    

    % Generated by detectors
    for i=1:length(obj.detectors)
        
        detectionLocation = [get_adr('2D_detections',obj.config) 'detections-' obj.detectors{i}.name '/'];
        detectionFilenames = strcat(detectionLocation,imageNames);
        detectionFilenames = strcat(detectionFilenames,'.txt');
        obj.detectors{i}.filenames=detectionFilenames;
        
    end
    clear loadedData;

    % Label each image
    nImages = length(files);
    for i = 1:nImages
        disp(['Labeling image ' num2str(i) '/' num2str(nImages) '...']);
        
        detectionDataImg = [];
        for d=1:length(obj.detectors)
            detectionDataImg(d).name=obj.detectors{d}.name;
            detectionDataImg(d).class=obj.detectors{d}.class;
            detectionDataImg(d).crfWeight=obj.detectors{d}.crfWeight;
            detectionDataImg(d).filenames=obj.detectors{d}.filenames{i};
        end
        
        %Do the actual labeling
        LabelOneImage(obj, imageNames{i}, files(i), detectionDataImg);
    end
    
end

function LabelOneImage(obj,imageName,files,detectionData)

    if ~exist(files.imageFilename,'file')
        warning('Image %s does not exist. Skipping...',imageFilename);
        return;
    end
    
    % Rectified image
    image = imread(files.imageFilename);

    height = size(image,1);
    width = size(image,2);
    
    % Original image - before rectification
    imOrig = imread(files.origImageFilename);
    
    % Un-rectified image size
    heightOrig = size(imOrig,1);
    widthOrig = size(imOrig,2);
    
    %% First layer
    segmentation = dlmread(files.segFilename);
    % Since segments start from 0
    segmentation = segmentation + 1;   

    % Number of segments in the image
    numSegs = max(segmentation(:));
    
    % New SVM testing
    prob_estimates = dlmread(files.classificationFilename);

    if length(prob_estimates)~=numSegs
        error(['Image ' imageName ': segment count mismatch!']);
    end

    % ---- Label probability maps   

    nClasses = size(prob_estimates,2) - 1;
    segMap = zeros(size(segmentation,1),size(segmentation,2),nClasses);
    % Create a probability map for each pixel
    for c=1:nClasses
        cVec = prob_estimates(:,c+1);
        segMap(:,:,c) = cVec(segmentation);
    end
    

    % Superpixel output
    [~,oldImg] = max(segMap,[],3);
    
    %%%%%%%%%%%%%%%%%%
    %% Second layer %%
    %%%%%%%%%%%%%%%%%%  
    %% Detectors

    if ~isempty(detectionData(1).filenames)
        nDetectors = length(detectionData);

            for i=1:nDetectors
                detectionFilename = detectionData(i).filenames;

                if ~isempty(detectionFilename)
                    s = dir(detectionFilename);
                    if s.bytes == 0
                        warning(['No ' detectionData(i).name ' detections found in this image.']);
                        dets =[];
                    else
                        dets = dlmread(detectionFilename);
                    end;   
                else
                    dets =[];
                end

                detectionMap = 1/obj.nClasses* ones(height,width,obj.nClasses);

                numDetections = size(dets,1);

                rangeScores = [min(dets(:,5)) max(dets(:,5))];
                rangeProbs = [1/obj.nClasses (1-0.001*(obj.nClasses-1))];
                for d = 1:numDetections
                    detectionRect = dets(d,1:4);  %Detection bounding box rectangle
                    detectionScore = dets(d,5);   %Score of the detection
                    
                    % Validation set-based mapping
                    % [~,pos] =min(abs([labelMaps.score]-detectionScore));

                    % Linear mapping
                    relScore = (detectionScore-min(rangeScores))/(max(rangeScores)-min(rangeScores));
                    classProb = min(rangeProbs) + relScore* (max(rangeProbs)-min(rangeProbs));
                    
                    % Max mapping
                    % classProb = max(rangeProbs);
                    
                    nonClassProb = (1-classProb)/(obj.nClasses-1);
                    
                    startRow = max(round(detectionRect(2)),1);
                    endRow = min(round(detectionRect(4)),height);
                    startColumn = max(round(detectionRect(1)),1);
                    endColumn = min(round(detectionRect(3)),width);
                    
                    detHeight = endRow-startRow+1;
                    detWidth = endColumn-startColumn+1;
                    
                    if (endRow-startRow<1) || (endColumn-startColumn<1)
                     continue;
                    end

                    probMap = zeros(detHeight,detWidth,obj.nClasses);
                    for c=1:obj.nClasses
                        if c~=detectionData(i).class
                            probMap(:,:,c) = nonClassProb * ones(detHeight,detWidth);
                        else
                            probMap(:,:,c) = classProb * ones(detHeight,detWidth);
                        end
                    end

                    detectionMap(startRow:endRow,startColumn:endColumn,:) = probMap;
                end   
                detectionData(i).detectionMap = detectionMap;
            end

    else
        nDetectors = 0;
    end


    %% Unrectification
    
    scale = dlmread(files.scaleFilename);
    homography = dlmread(files.rectFilename);
    
    resizedImage = imresize(image,scale);
    
    unrectifiedImage = zeros(heightOrig,widthOrig,3);
    for i=1:3
        unrectifiedImage(:,:,i) = rewarp(imOrig,resizedImage(:,:,i),homography);
    end

    segMapUnrect = zeros(heightOrig,widthOrig,obj.nClasses);
    for i=1:nDetectors
        detectionData(i).detectionMapUnrect = zeros(heightOrig,widthOrig,obj.nClasses);
    end

    for i=1:obj.nClasses
        s2 = imresize(segMap(:,:,i),scale,'nearest');
        segMapUnrect(:,:,i)=rewarp(unrectifiedImage(:,:,1),s2,homography,1/obj.nClasses);
        for j=1:nDetectors
            d2 = imresize(detectionData(j).detectionMap(:,:,i),scale,'nearest'); 
            detectionData(j).detectionMapUnrect(:,:,i) = rewarp(unrectifiedImage(:,:,1),d2,homography,1/obj.nClasses);
        end
    end
    segMap = segMapUnrect;
    for i=1:nDetectors
        detectionData(i).detectionMap = detectionData(i).detectionMapUnrect;
    end
    clear segMapUnrect;

    image = uint8(unrectifiedImage);
        
    %% Run CRF
    [outImg,~,~] = RunCRF(segMap,detectionData,obj.nClasses,obj.crf);

    %% Output
    outputDir = get_adr('2D_classification',obj.config,obj.classifierName);
    
    % Save overlayed result image to disk
    mkdirIfNotExist([outputDir '/layer1/']);
    mkdirIfNotExist([outputDir '/layer2/']);
        
    oldImg = imresize(oldImg,scale,'nearest');
    oldImg = rewarp(imOrig,oldImg,homography);
    selector=1;
    writeSegmentationToDisk(oldImg,[outputDir '/layer1/' imageName '_overlay.png'],image,0.5,2,selector);
    writeSegmentationToDisk(outImg,[outputDir '/layer2/' imageName '_overlay.png'],image,0.5,2,selector);
    
    writeSegmentationToDisk(oldImg,[outputDir '/layer1/' imageName '.png'],image,1,2,selector);
    writeSegmentationToDisk(outImg,[outputDir '/layer2/' imageName '.png'],image,1,2,selector);
    
    for i=1:nDetectors
        dmap = detectionData(i).detectionMap(:,:,detectionData(i).class);
        writeSegmentationToDisk(dmap,[outputDir '/layer2/' imageName '_detections_' detectionData(i).name '.png'],image,0.5,1,selector);
    end
    
    % A big mat file
    segPotentials = segMap;
    layer1_labeling = oldImg;
    layer2_labeling = outImg;
    
    save(get_adr('2D_classifier_unaries',obj.config,obj.classifierName,imageName),'segPotentials','detectionData','layer1_labeling','layer2_labeling');

end
