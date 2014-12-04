function RunThirdLayerSingle(facadeID, dirName, baseName, splitName, hyperSetup)
    
    % Hyper-parameters?
    hyperParameters = SetupHyperParameters(hyperSetup);
    
    if hyperParameters.parallel
        
        poolobj = gcp('nocreate'); % If pool doesnt exist, do not create a new one.
        if isempty(poolobj)
            parpool(12);
        end
    end
   
    dirName = [dirName 'work/pcl/split/' baseName '_' splitName '/'];

    imgName = [baseName '_split_' num2str(facadeID) '_ortho_colors.png'];
    labName = [baseName '_split_' num2str(facadeID) '_ortho_labeling.png'];
    potName = [baseName '_split_' num2str(facadeID) '_ortho_potentials.mat'];
    
    % Labeled image
    labImage = imread([dirName labName]);
    outImg = Image2Label(labImage);
        
    % Original image
    origImg = imread([dirName imgName]);
    origImg = double(origImg)/256;
        
    % Per-pixel probabilities
    ss = load([dirName potName]);
    segMap = ss.orthoPotentials;
    segMap  = exp(-segMap);
           
     % Run third layer
    thirdLayerOutput = elementSampling(origImg,segMap,outImg,hyperParameters);

    if hyperParameters.visualize
        figure(300);
        subplot(121);imagesc(outImg);axis equal;
        subplot(122);imagesc(thirdLayerOutput);axis equal;
    end

    thirdLayerImage = Label2Image(thirdLayerOutput);

    imwrite(thirdLayerImage,[dirName baseName '_split_' num2str(facadeID) '_ortho_labeling_layer3.png']);
        
    if hyperParameters.parallel
        delete(gcp);
    end

end