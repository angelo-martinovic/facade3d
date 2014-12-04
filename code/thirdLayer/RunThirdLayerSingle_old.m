function RunThirdLayerSingle(facadeID, dirName, baseName, splitName, hyperSetup)
    
    % Hyper-parameters?
    hyperParameters = SetupHyperParameters(hyperSetup);
    rectificationLoc = '/esat/sadr/amartino/Code/rectification/';
    
    if hyperParameters.parallel
        
        poolobj = gcp('nocreate'); % If no pool, do not create new one.
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
           
    if hyperParameters.rectification
        cmd = ['python ' rectificationLoc 'rectifyImage.py -t auto -i ' ...
            dirName  imgName ' -o ' ...
            dirName  imgName(1:end-4) '_rect.png'];

        % Run rectification if it hasn't been run before
        rectImageFileName = [dirName  imgName(1:end-4) '_rect.png'];
        if ~exist(rectImageFileName,'file')
            [rectFailed,output] = system(cmd,'-echo');
        else
            rectFailed = 0;
        end

        if ~rectFailed
            % Rectified image
            rectImg = imread(rectImageFileName);

            % Rectification parameters
            homography = dlmread([dirName imgName(1:end-4) '_rect_rect.dat']);
            homography = reshape(inv(reshape(homography,3,3)),1,9);

            outImgRect = rewarp(rectImg,outImg,homography);

            origImgRect = [];
            for ch=1:3
                origImgRect(:,:,ch) = rewarp(rectImg,origImg(:,:,ch),homography);
            end

            segMapRect = [];
            for ch=1:size(segMap,3)
                segMapRect(:,:,ch) = rewarp(rectImg,segMap(:,:,ch),homography);
            end

            clear lStruct;
        
            % Run third layer
            thirdLayerOutput = elementSampling(origImgRect,segMapRect,outImgRect,hyperParameters);

            % Un-rectify
            homography = reshape(inv(reshape(homography,3,3)),1,9);
            
            thirdLayerOutput = rewarp(origImg,thirdLayerOutput,homography);
        else
            % Run third layer as is
            thirdLayerOutput = elementSampling(origImg,segMap,outImg,hyperParameters);
        end
    else
         % Run third layer
        thirdLayerOutput = elementSampling(origImg,segMap,outImg,hyperParameters);
    end

    if hyperParameters.visualize
        figure(300);
        subplot(121);imagesc(outImg);axis equal;
        subplot(122);imagesc(thirdLayerOutput);axis equal;
    end

    thirdLayerImage = Label2Image(thirdLayerOutput);

    imwrite(thirdLayerImage,[dirName baseName '_split_' num2str(facadeID) '_ortho_labeling_layer3.png']);
        
%     thirdLayerTime = toc;
%     fprintf('Elapsed %d seconds.\n',thirdLayerTime);
    if hyperParameters.parallel
        delete(gcp);
    end

end