% Type is train, valid, or eval
% [t,x] : t are target values, x are input values
function [t,x,segsPerImage,imageNames] = LoadData(obj,subset)
    dl = DispatchingLogger.getInstance();
    imageNames = obj.LoadFilenames(subset);
    
    % Empty set
    if (isempty(imageNames))
        t=[]; x=[];
        segsPerImage=[]; 
        dl.Log(VerbosityLevel.Warning,sprintf('Data not found'));
        return;
    end
    
    allSegDistributions = zeros(0,obj.config.nClasses);
    allSegs = [];
    
    segsPerImage= cell(0);
    missingFiles = zeros(1,length(imageNames));
    for i=1:length(imageNames)
        segName = get_adr('2D_segmentation',obj.config,imageNames{i});
        if ~exist(segName,'file')
            dl.Log(VerbosityLevel.Warning,sprintf('Image %d segmentation not found\n',i));
            missingFiles(i) = 1;
            continue;
        end
        
        segmentation = dlmread(segName);
        
        gtFile = get_adr('2D_label',obj.config,imageNames{i});
        if exist(gtFile, 'file')
            gt = imread(gtFile);
            groundTruth = Image2Labels(double(gt), obj.config.cm);
        else
            dl.Log(VerbosityLevel.Warning,sprintf('Image %d ground truth not found\n',i));
            groundTruth = zeros(size(segmentation));
        end
        features = [];
        for f=1:length(obj.config.c2D.featureExtractors)
            featureName = obj.config.c2D.featureExtractors{f}.name;
            feats_f = load(get_adr('2D_features',obj.config,imageNames{i},featureName));
            features = [features feats_f.features];
        end
        
        % Scale
        sc = dlmread(get_adr('2D_scale',obj.config,imageNames{i}));
        target = zeros(sc(1),sc(2));
        
        % Rectify ground truth
        h = dlmread(get_adr('2D_rectParams',obj.config,imageNames{i}));
        h = reshape(inv(reshape(h,3,3)),1,9);
        groundTruth = rewarp(target,groundTruth,h);
        groundTruth = imresize(groundTruth,size(segmentation),'nearest');
       
         % Since segments start from 0
        segmentation = segmentation + 1;   
    
        % Number of segments in the image
        numSegs = max(segmentation(:));
        
        segDistributions = zeros(numSegs,obj.config.nClasses);
        for r=1:numSegs
            res2 = groundTruth(segmentation==r);  
            label = mode(res2);
            % If label is void, use second best
            if label==0
               res2(res2==0)=[];
               label = mode(res2);
            end
            if (label>=1 && label<=obj.config.nClasses)
                segDistributions(r,label)=1;
            end

        end
        voidIndices = (sum(segDistributions,2)==0);
        
        % Ignore void-labeled segments in training set
        if strcmp(subset,'train')
            segDistributions = segDistributions(~voidIndices,:);
            features = features(~voidIndices,:);
            segsPerImage{end+1}= sum(~voidIndices); 
        else
            segsPerImage{end+1}= numSegs;
        end
        
        allSegDistributions = [allSegDistributions; segDistributions];
        allSegs = [allSegs; features];
        
    end

    imageNames = imageNames(missingFiles==0);
    t = allSegDistributions;
    t = bsxfun(@rdivide,t,sum(t,2));
    clear allSegDistributions;

    x = allSegs;
    clear allSegs;
end
