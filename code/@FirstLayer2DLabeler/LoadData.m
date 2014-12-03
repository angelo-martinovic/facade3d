% Type is train, valid, or eval
% [t,x] : t are target values, x are input values
function [t,x,segsPerImage,imageNames] = LoadData(obj,subset)

    imageNames = obj.LoadFilenames(subset);
    
    % Empty set
    if (isempty(imageNames))
        t=[]; x=[];
        segsPerImage=[]; 
        warning('Data not found.');
        return;
    end
    
    origImageNames = strcat([obj.dirName 'work/'],imageNames);
    
    segFilenames = strcat(origImageNames,'.seg');
    groundTruthFilenames = strcat(strcat([obj.dirName 'labels/'],imageNames),'.png');
    featureFilenames = strcat(origImageNames,'.features.txt');
    scaleFilenames = strcat(origImageNames,'_scale.dat');
    rectFilenames = strcat(origImageNames,'_rect.dat');
    
    allSegDistributions = zeros(0,obj.nClasses);
    allSegs = zeros(0,obj.nFeats);
    
    segsPerImage= cell(0);
    pb = ProgressBar(length(segFilenames));
    missingFiles = zeros(1,length(segFilenames));
    for i=1:length(segFilenames)
        if ~exist(segFilenames{i},'file')
            warning('Image %d not found',i);
            missingFiles(i) = 1;
            pb.progress;
            continue;
        end
        
        segmentation = dlmread(segFilenames{i});
        if exist(groundTruthFilenames{i}, 'file')
            groundTruth = imread(groundTruthFilenames{i});
            groundTruth = Image2Label(groundTruth);
        else
            warning('No ground truth');
            groundTruth = zeros(size(segmentation));
        end
        features = dlmread(featureFilenames{i});
        
        % Scale
        sc = dlmread(scaleFilenames{i});
        target = zeros(sc(1),sc(2));
        
        % Rectify ground truth
        h = dlmread(rectFilenames{i});
        h = reshape(inv(reshape(h,3,3)),1,9);
        groundTruth = rewarp(target,groundTruth,h);
        groundTruth = imresize(groundTruth,size(segmentation),'nearest');
       
         % Since segments start from 0
        segmentation = segmentation + 1;   
    
        % Number of segments in the image
        numSegs = max(segmentation(:));
        
        segDistributions = zeros(numSegs,obj.nClasses);
        for r=1:numSegs
            res2 = groundTruth(segmentation==r);  
            label = mode(res2);
            % If label is void, use second best
            if label==0
               res2(res2==0)=[];
               label = mode(res2);
            end
            if (label>=1 && label<=obj.nClasses)
                segDistributions(r,label)=1;
            end

        end
        voidIndices = (sum(segDistributions,2)==0);
        
        if strcmp(subset,'train')
            segDistributions = segDistributions(~voidIndices,:);
            features = features(~voidIndices,:);
            segsPerImage{end+1}= sum(~voidIndices); 
        else
            segsPerImage{end+1}= numSegs;
        end
        
        allSegDistributions = [allSegDistributions; segDistributions];
        allSegs = [allSegs; features];
        
        pb.progress;
    end
    pb.stop;
    disp('Done');

    imageNames = imageNames(missingFiles==0);
    t = allSegDistributions';
    t = bsxfun(@rdivide,t,sum(t));
    clear allSegDistributions;

    x = allSegs';
    clear allSegs;
end
