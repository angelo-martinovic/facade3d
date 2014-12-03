function BatchAugmentFeaturesWithDepth

    dirname = '/esat/sadr/amartino/monge3d/reconstruction.nvm.cmvs/00/visualizeRect/';
    
    filenames = dir([dirname '*.features.txt']);
    
    depthSums = zeros(1,8);
    depthCounts = zeros(1,8);
    for i=1:length(filenames)
        fprintf('.');
        filename = filenames(i).name;
        features = dlmread([dirname filename]);
        segmentation = dlmread([dirname filename(1:end-13) '.seg']);
        depthMap = dlmread([dirname filename(1:end-13) '_depth.txt']);
        gt = dlmread([dirname filename(1:end-13) '.txt']);
        
        % Comment the following 3 lines if no rectification
%         homography = dlmread([dirname filename(1:end-13) 'rect.dat']);
%         homography = reshape(inv(reshape(homography,3,3)),1,9);
%         gt = rewarp(segmentation,gt,homography);
        
        optimumLabeling = zeros(size(gt));
        featuresAugmented = zeros(size(features,1),size(features,2)+1);
        disp(size(featuresAugmented));
        for seg = 1:max(segmentation(:))
            segDepths = depthMap(segmentation==seg);
            depthFeature = mean(segDepths);
            
            featuresAugmented(seg,:) = [features(seg,:) depthFeature];
            
            segGt = gt(segmentation==seg);
            label = mode(segGt);
            
            optimumLabeling(segmentation==seg)=label;
            if label>=1
            
                depthSums(label) = depthSums(label)+depthFeature;
                depthCounts(label) = depthCounts(label)+1;
            end
           
        end
        figure(1);imagesc(optimumLabeling);
        
        
        movefile([dirname filename],[dirname 'featuresNoDepth/' filename]);
        dlmwrite([dirname filename(1:end-13) '.features.txt'],featuresAugmented,' ');
        
        
    end
    depthHist = depthSums./depthCounts;
    
    figure;bar(depthHist);
    
    save('depths.mat','depthHist');
    
    

end