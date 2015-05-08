% Type is train, valid, or eval
% [t,x] : t are target values, x are input values
function [t,x,segsPerImage,imageNames] = LoadData(subset)
    dl = DispatchingLogger.getInstance();
    cf = DatasetConfig.getInstance();
    
    imageNames = LoadFilenames(subset);
    
    % Empty set
    if (isempty(imageNames))
        t=[]; x=[];
        segsPerImage=[]; 
        dl.Log(VerbosityLevel.Warning,sprintf('Data not found'));
        return;
    end
    
    allSegDistributions = zeros(0,cf.nClasses);
    allSegs = [];
    
    pb = ProgressBar(length(imageNames));
    
    segsPerImage= cell(0);
    missingFiles = zeros(1,length(imageNames));
    for i=1:length(imageNames)
        segName = get_adr('2D_segmentation',imageNames{i});
        if ~exist(segName,'file')
            dl.Log(VerbosityLevel.Warning,sprintf('Image %d segmentation not found\n',i));
            missingFiles(i) = 1;
            pb.progress();
            continue;
        end
        
        segmentation = dlmread(segName);
        
        gtFile = get_adr('2D_label',imageNames{i});
        if exist(gtFile, 'file')
            gt = imread(gtFile);
            groundTruth = Image2Labels(double(gt), cf.cm);
        else
            dl.Log(VerbosityLevel.Warning,sprintf('Image %d ground truth not found\n',i));
            groundTruth = zeros(size(segmentation));
        end
        features = [];
        for f=1:length(cf.c2D.featureExtractors)
            featureName = cf.c2D.featureExtractors{f}.name;
            feats_f = load(get_adr('2D_features',imageNames{i},featureName));
            features = [features feats_f.features];
        end
        
        % Scale
        scaleFilename = get_adr('2D_scale',imageNames{i});
        if exist(scaleFilename,'file')
            sc = dlmread(scaleFilename);
            target = zeros(sc(1),sc(2));
        else
            target = zeros(size(segmentation));
        end
        
        % Rectify ground truth
        rectParamsFilename = get_adr('2D_rectParams',imageNames{i});
        if exist(rectParamsFilename,'file')
            h = dlmread(rectParamsFilename);
            h = reshape(inv(reshape(h,3,3)),1,9);
            groundTruth = rewarp(target,groundTruth,h);
            groundTruth = imresize(groundTruth,size(segmentation),'nearest');
        end
       
         % Since segments start from 0
        segmentation = segmentation + 1;   
    
        % Number of segments in the image
        numSegs = max(segmentation(:));
        
        segDistributions = zeros(numSegs,cf.nClasses);
        for r=1:numSegs
            res2 = groundTruth(segmentation==r);  
            label = mode(res2);
            % If label is void, use second best
            if label==0
               res2(res2==0)=[];
               label = mode(res2);
            end
            if (label>=1 && label<=cf.nClasses)
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
        
        pb.progress();
    end
    pb.stop();

    imageNames = imageNames(missingFiles==0);
    t = allSegDistributions;
    t = bsxfun(@rdivide,t,sum(t,2));
    clear allSegDistributions;

    x = allSegs;
    clear allSegs;
end
