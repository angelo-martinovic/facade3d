% Gets all images, rectifies, segments, extracts features, runs detectors on test set.
function PrepareData()
    %% 1st layer - prepare data
    cf = DatasetConfig.getInstance();
    dl = DispatchingLogger.getInstance();

    % Create work folder
    workFolder = get_adr('work',cf);
    mkdirIfNotExist(workFolder);

    %% Get the list of all images
    subset = 'all';
    imageNames = LoadFilenames(subset);

    imageFilenames = strcat(get_adr('2D_images'),strcat(imageNames,'.jpg'));

    checkAdr_and_createDir('../tmp/');
    if cf.rectificationNeeded,
        %% Run rectification
        tic;

        dl.Log(VerbosityLevel.Info,'Rectifying images ...\n');
        parfor (i=1:length(imageFilenames),cf.nWorkers)
          cfLocal = DatasetConfig.getInstance(cf.name);
%         for (i=1:length(imageFilenames))
            outputImageName = get_adr('2D_image',imageNames{i});
            rectSkipName = get_adr('2D_rectSkip',imageNames{i});
            if ~exist(outputImageName,'file') && ~exist(rectSkipName,'file') 
                rectCmd = ['LD_LIBRARY_PATH= python rectification/rectifyImage.py -t auto -i '...
                    imageFilenames{i} ' -o ' outputImageName];

                [stat,res] = system(rectCmd);%,'-echo');
                if stat~=0
                    % Something went wrong
                    
                    % Check if it's opencv-python package
                    if ~isempty(strfind(res,'No module named cv2'))
                        dl.Log(VerbosityLevel.Warning,'Package opencv-python not found! Attempting to run on sadr...\n'); %#ok<PFBNS>
                        
                        % If package missing, try on sadr
                        rectCmd = ['ssh sadr "cd ' pwd ' && ' rectCmd '"'];
                        [stat,res] = system(rectCmd);%,'-echo');
                        
                        
                         if stat~=0
                            % Fails even on sadr
                            dl.Log(VerbosityLevel.Warning,sprintf('Rectification failed on image %d with code %d and message %s\n',i,stat,res));
                         else
                             % Runs on sadr.
                             dl.Log(VerbosityLevel.Warning,...
                                 sprintf('Done. In the future, please install opencv-python locally...\n')); 
                         end
                    
                    else
                        % Rectification failed due to something else.
                        dl.Log(VerbosityLevel.Warning,sprintf('Rectification failed on image %d with code %d and message %s\n',i,stat,res));
                    end
                end
            end
            if ~exist(outputImageName,'file') && ~exist(rectSkipName,'file') 
                % Create a flag-file so that this image is
                % always skipped
                system(['touch ' rectSkipName]);
            end
            
        end
        dl.Log(VerbosityLevel.Info,sprintf('Rectified %d images in %.2f seconds.\n',length(imageFilenames),toc));
    else
        % Otherwise just copy
        tic;
        for i=1:length(imageFilenames)
            copyfile(imageFilenames{i}, get_adr('2D_image',imageNames{i}));
        end
        dl.Log(VerbosityLevel.Info,sprintf('Copied %d images in %.2f seconds.\n',length(imageFilenames),toc));
    end
    %% Remove empty images
    cntRemoved = 0;
    for i=1:length(imageFilenames)
        s = dir(get_adr('2D_image',imageNames{i}));
        deleteFlag = 0;
        if isempty(s), deleteFlag=1;end;
        if ~deleteFlag, if s.bytes==0, deleteFlag = 1; end; end
        
        if deleteFlag
            deleteIfExists(get_adr('2D_image',imageNames{i}));
            deleteIfExists(get_adr('2D_rectParams',imageNames{i}));
            deleteIfExists(get_adr('2D_scale',imageNames{i}));
            system(['touch ' get_adr('2D_rectSkip',imageNames{i})]);
            cntRemoved = cntRemoved+1;
        end
    end
    dl.Log(VerbosityLevel.Info,sprintf('Removed %d empty images.\n',cntRemoved));
    
    [~,nExisting]=system(['ls ' workFolder '/*.ppm | wc -l']);
    nExisting = str2double(nExisting);
    if nExisting~=length(imageFilenames)-cntRemoved
        %% Resize images
        if cf.resizeImages
            tic;
            dl.Log(VerbosityLevel.Info,sprintf('Resizing images to common height: %d ...\n',cf.c2D.resizedImagesHeight));
            resizeCmd = ['mogrify -resize x' num2str(cf.c2D.resizedImagesHeight) ' ' workFolder '*.jpg'];
            [stat,res] = system(resizeCmd); %,'-echo');
            if stat~=0
                error('Error running mogrify resize: %s\n',res);
            end
            dl.Log(VerbosityLevel.Info,sprintf('Image resize time: %.2f seconds.\n',toc));
        end
    
        %% Convert to ppm
        tic;
        dl.Log(VerbosityLevel.Info,sprintf('Converting to ppm (needed for meanshift) ...\n'));
        convertCmd = ['mogrify -format ppm ' workFolder '*.jpg'];
        [stat,res] = system(convertCmd); %,'-echo'); % add for debug
        if stat~=0
            dl.Log(VerbosityLevel.Error,sprintf('Error running mogrify convert: %s\n',res));
            fatal();
        end
        dl.Log(VerbosityLevel.Info,sprintf('Elapsed time: %.2f seconds.\n',toc));
    end
    
    %% Run meanshift
    tic;
    dl.Log(VerbosityLevel.Info,sprintf('Meanshift segmentation ...\n'));
    checkAdr_and_createDir('edison/tmp/');
    parfor (i=1:length(imageFilenames),cf.nWorkers)
        if ~exist(get_adr('2D_segmentation',imageNames{i}),'file')
            
            ppmFile = get_adr('2D_ppm',imageNames{i});
            if exist(ppmFile,'file')
                copyfile(ppmFile,sprintf('edison/tmp/%08d.ppm',i));
                segCmd = sprintf('edison/segmentImage.sh %08d.ppm %d',i,cf.c2D.minRegionArea); 
                [stat,res] = system(segCmd); %,'-echo'); % add for debug
                if stat~=0
                    dl.Log(VerbosityLevel.Error,sprintf('Error running segmentImage: %s\n',res)); %#ok<PFBNS>
                    fatal();
                end
                delete(sprintf('edison/tmp/%08d.ppm',i));
                movefile(sprintf('edison/tmp/%08d.seg',i),[ppmFile(1:end-4) '.seg']);
            end
        end
    end
    dl.Log(VerbosityLevel.Info,sprintf('Elapsed time: %.2f seconds.\n',toc));

    %% Run feature extractor(s)
    featureExtractors = cf.c2D.featureExtractors;
    nFeatExtr = length(featureExtractors);
    if nFeatExtr<1
        dl.Log(VerbosityLevel.Error,sprintf('No feature extractors defined!'));
        fatal();
    end
    dl.Log(VerbosityLevel.Info,sprintf('Running %d feature extractors...\n',nFeatExtr));
    for i=1:nFeatExtr
        dl.Log(VerbosityLevel.Info,sprintf('Running feature extractor: %s ...\n',featureExtractors{i}.name));
        tic;
        try
            featureExtractors{i}.ExtractFeatures();
        catch ME
            dl.Log(VerbosityLevel.Error,ME.message);
            fatal();
        end
        dl.Log(VerbosityLevel.Info,sprintf('Elapsed time for %s: %.2f seconds.\n',...
            featureExtractors{i}.name,toc));
    end
   
    %% Run detector(s)
    imageNamesTrain = LoadFilenames('train');
    imageNamesEval = LoadFilenames('eval');
    dl.Log(VerbosityLevel.Info,sprintf('Running %d detectors...\n',length(cf.c2D.detectors)));
    for d=1:length(cf.c2D.detectors)
        D = cf.c2D.detectors{d};
        
        % Copy all images to a subfolder (only once, before the first detector)
        if d==1
            subfolderEval = get_adr('2D_detectionFolderEval');
            if ~exist(subfolderEval,'dir')
                mkdir(subfolderEval);
            else
                if cf.useCache
                    dl.Log(VerbosityLevel.Info,sprintf('Found cache in %s. Skipping detector %s ...\n',subfolderEval,D.name));
                    continue;
                else
                   system(['rm -r ' subfolderEval]);  %,'-echo'); % add for debug
                   mkdir(subfolderEval);
                end
            end

            for i=1:length(imageNamesEval)
                 if exist(get_adr('2D_image',imageNamesEval{i}),'file')
                   copyfile(get_adr('2D_image',imageNamesEval{i}),subfolderEval);
                 end
            end
            
            subfolderTrain = get_adr('2D_detectionFolderTrain');
            if ~exist(subfolderTrain,'dir')
                mkdir(subfolderTrain);
            else
               system(['rm -r ' subfolderTrain]);  %,'-echo'); % add for debug
               mkdir(subfolderTrain);
            end

            for i=1:length(imageNamesTrain)
                 if exist(get_adr('2D_image',imageNamesTrain{i}),'file')
                   copyfile(get_adr('2D_image',imageNamesTrain{i}),subfolderTrain);
                 end
            end
        end

        % Run detector on the eval subfolder
        dl.Log(VerbosityLevel.Info,sprintf('Running detector %s ...\n',D.name));
        outputFolder = get_adr('2D_detectorOutputFolderEval',D.name);
        tic;
        try
            D.DetectObjects(subfolderEval,outputFolder);
        catch ME
            dl.Log(VerbosityLevel.Error,ME.message);
            fatal();
        end
        dl.Log(VerbosityLevel.Info,sprintf('Detector %s elapsed time: %.2f seconds.\n',D.name,toc));
        
        % Create the mean detection on the train subfolder
        dl.Log(VerbosityLevel.Info,sprintf('Creating mean detection file with detector %s ...\n',D.name));
        outputFolder = get_adr('2D_detectorOutputFolderTrain',D.name);
        tic;
        try
            D.DetectObjects(subfolderTrain,outputFolder);
            D.CalculateMeanDetection(imageNamesTrain);
        catch ME
            dl.Log(VerbosityLevel.Error,ME.message);
            fatal();
        end
        dl.Log(VerbosityLevel.Info,sprintf('Detector %s elapsed time: %.2f seconds.\n',D.name,toc));
    
        % Remove all images from the subfolder (only once, after the last detector)
        if d==length(cf.c2D.detectors)
          
          for i=1:length(imageNamesTrain)
              delete([subfolderTrain imageNamesTrain{i} '.jpg']);
              
          end
          for i=1:length(imageNamesEval)
              delete([subfolderEval imageNamesEval{i} '.jpg']);
          end 
        end 
    end

end
