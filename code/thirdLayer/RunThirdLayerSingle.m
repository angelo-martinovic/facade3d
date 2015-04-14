function RunThirdLayerSingle(splitName,facadeID,hyperSetup)
    facadeID = num2str(facadeID);
    
    % Read config from temp file
    load('datasetConfig.mat');
    
    % Set-up the hyper-parameters of the third layer
    hyperParameters = SetupHyperParameters(hyperSetup);
    
%     if datasetConfig.nWorkers>1
%         InitializeParallel(datasetConfig.nWorkers);
%     end

    % Labeled image
    labImage = imread(get_adr('orthoLabels',datasetConfig,splitName,facadeID));
    outImg = Image2Labels(double(labImage),datasetConfig.cm);
        
    % Original image
    origImg = imread(get_adr('orthoColors',datasetConfig,splitName,facadeID));
    origImg = double(origImg)/256;
        
    % Per-pixel probabilities
    ss = load(get_adr('orthoPotentials',datasetConfig,splitName,facadeID));
    segMap = ss.orthoPotentials;
    segMap  = exp(-segMap);
           
     % Run third layer
    thirdLayerOutput = elementSampling(origImg,segMap,outImg,hyperParameters);

    if hyperParameters.visualize
        figure(300);
        subplot(121);imagesc(outImg);axis equal;
        subplot(122);imagesc(thirdLayerOutput);axis equal;
    end

    thirdLayerImage = Label2Image(thirdLayerOutput,datasetConfig.cm);

    imwrite(thirdLayerImage,get_adr('orthoLabelingLayer3Img',datasetConfig,splitName,facadeID));
        
end