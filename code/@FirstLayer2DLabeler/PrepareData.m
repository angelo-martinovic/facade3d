% Gets all images, rectifies, segments, extracts features, runs detectors on test set.
function PrepareData(obj)
    %% 1st layer - prepare data
    cf = obj.config;
    dl = DispatchingLogger.getInstance();

    % Create work folder
    workFolder = get_adr('work',obj.config);
    mkdirIfNotExist(workFolder);

    %% Get the list of all images
    subset = 'all';
    imageNames = obj.LoadFilenames(subset);

    imageFilenames = strcat(get_adr('2D_images',cf),strcat(imageNames,'.jpg'));

    if cf.rectificationNeeded,
        %% Run rectification
        tic;

        dl.Log(VerbosityLevel.Info,'Rectifying images ...\n');
        parfor (i=1:length(imageFilenames),cf.nWorkers)
%         for (i=1:length(imageFilenames))
            outputImageName = get_adr('2D_image',cf,imageNames{i});
            rectSkipName = get_adr('2D_rectSkip',cf,imageNames{i});
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
        rectificationTime = toc;
        dl.Log(VerbosityLevel.Info,sprintf('Rectified %d images in %.2f seconds.\n',length(imageFilenames),rectificationTime));
    else
        % Otherwise just copy
        tic;
        for i=1:length(imageFilenames)
            copyfile(imageFilenames{i},[workFolder get_adr('2D_image',cf,imageNames{i})]);
        end
        copyTime = toc;
        dl.Log(VerbosityLevel.Info,sprintf('Copied %d images in %.2f seconds.\n',length(imageFilenames),copyTime));
    end
    %% Remove empty images
    cntRemoved = 0;
    for i=1:length(imageFilenames)
        s = dir(get_adr('2D_image',cf,imageNames{i}));
        deleteFlag = 0;
        if isempty(s), deleteFlag=1;end;
        if ~deleteFlag, if s.bytes==0, deleteFlag = 1; end; end
        
        if deleteFlag
            deleteIfExists(get_adr('2D_image',cf,imageNames{i}));
            deleteIfExists(get_adr('2D_rectParams',cf,imageNames{i}));
            deleteIfExists(get_adr('2D_scale',cf,imageNames{i}));
            system(['touch ' get_adr('2D_rectSkip',cf,imageNames{i})]);
            cntRemoved = cntRemoved+1;
        end
    end
    dl.Log(VerbosityLevel.Info,sprintf('Removed %d empty images.\n',cntRemoved));
    
    %% Resize images
    tic;
    dl.Log(VerbosityLevel.Info,sprintf('Resizing images to common height: %d ...\n',cf.c2D.resizedImagesHeight));
    resizeCmd = ['mogrify -resize x' num2str(cf.c2D.resizedImagesHeight) ' ' workFolder '*.jpg'];
    [stat,res] = system(resizeCmd); %,'-echo');
    if stat~=0
        error('Error running mogrify resize: %s\n',res);
    end
    resizeTime = toc;
    dl.Log(VerbosityLevel.Info,sprintf('Image resize time: %.2f seconds.\n',resizeTime));
    
    %% Convert to ppm
    tic;
    dl.Log(VerbosityLevel.Info,sprintf('Converting to ppm (needed for meanshift) ...\n'));
    convertCmd = ['mogrify -format ppm ' workFolder '*.jpg'];
    [stat,res] = system(convertCmd); %,'-echo'); % add for debug
    if stat~=0
        dl.Log(VerbosityLevel.Error,sprintf('Error running mogrify convert: %s\n',res));
        error('Critical error. Terminating.');
    end
    convertTime = toc;
    dl.Log(VerbosityLevel.Info,sprintf('Mogrify convert time: %f seconds.\n',convertTime));
    
    %% Run meanshift
    tic;
    dl.Log(VerbosityLevel.Info,sprintf('Meanshift segmentation ...\n'));
    parfor (i=1:length(imageFilenames),cf.nWorkers)
        if ~exist(get_adr('2D_segmentation',cf,imageNames{i}),'file')
            
            ppmFile = get_adr('2D_ppm',cf,imageNames{i});
            if exist(ppmFile,'file')
                copyfile(ppmFile,'edison/tmp/');
                segCmd = ['edison/segmentImage.sh ' imageNames{i} '.ppm' ' '...
                    num2str(cf.c2D.minRegionArea)];
                [stat,res] = system(segCmd); %,'-echo'); % add for debug
                if stat~=0
                    dl.Log(VerbosityLevel.Error,sprintf('Error running segmentImage: %s\n',res)); %#ok<PFBNS>
                    error('Critical error. Terminating.');
                end
                delete(['edison/tmp/' imageNames{i} '.ppm']);
                movefile(['edison/tmp/' imageNames{i} '.seg'],workFolder);
            end
        end
    end
    segmentationTime = toc;
    dl.Log(VerbosityLevel.Info,sprintf('Meanshift elapsed time: %.2f seconds.\n',segmentationTime));

    %% Run feature extractor(s)
    featureExtractors = cf.c2D.featureExtractors;
    if length(featureExtractors)<1
        dl.Log(VerbosityLevel.Error,sprintf('No feature extractors defined!'));
        error('Critical error. Terminating.');
    end
    for i=1:length(featureExtractors) 
        dl.Log(VerbosityLevel.Info,sprintf('Running feature extractor: %s ...\n',featureExtractors{i}.name));
        tic;
        featureExtractors{i}.ExtractFeatures();
        featureExtractionTime = toc;
        dl.Log(VerbosityLevel.Info,sprintf('Elapsed time for %s: %.2f seconds.\n',...
            featureExtractors{i}.name,featureExtractionTime));
    end
   
    %% Run detector(s)
    subset = 'eval';
    imageNames = obj.LoadFilenames(subset);
    
    for d=1:length(cf.c2D.detectors)
        D = cf.c2D.detectors{d};
        
        if d==1
            % Copy all images to a subfolder (only once)
            subfolder = get_adr('2D_detectionFolder',cf);
            if ~exist(subfolder,'dir')
                mkdir(subfolder);
            else
                if cf.useCache
                    dl.Log(VerbosityLevel.Info,sprintf('Found cache. Skipping detector %s ...\n',D.name));
                    continue;
                else
                    system(['rm -r ' subfolder]);  %,'-echo'); % add for debug
                    mkdir(subfolder);
                end
            end

            for i=1:length(imageNames)
                 if exist(get_adr('2D_image',cf,imageNames{i}),'file')
                    copyfile(get_adr('2D_image',cf,imageNames{i}),subfolder);
                 end
            end
        end

        dl.Log(VerbosityLevel.Info,sprintf('Running detector %s ...\n',D.name));

        % Run detector on the subfolder
        tic;
        detCmd = ['LD_LIBRARY_PATH="" && external/biclop/src/applications/objects_detection/objects_detection -c ' ...
                    D.configFile ' --objects_detector.model ' D.modelFile ...
                    ' --recording_path '  subfolder  ' --process_folder '  subfolder ];
        [stat,res] = system(detCmd);   %,'-echo'); % add for debug
        if stat~=0
            dl.Log(VerbosityLevel.Error,sprintf('Error running object detector: %s\n',res));
        end

        dl.Log(VerbosityLevel.Info,sprintf('Creating detection files ...\n'));
        detCmd2 = ['python external/biclop/tools/objects_detection/detections_to_wnd_eval_normalized.py -d ' subfolder ...
            'detections.data_sequence -o ' get_adr('2D_detectorOutputFolder',cf,D.name) ' -m ' D.modelFile];
        [stat,res] = system(detCmd2);%,'-echo'); % add for debug
        if stat~=0
            dl.Log(VerbosityLevel.Error,sprintf('Error creating detection files: %s\n',res));
        end
        delete([subfolder 'detections.data_sequence']); 
        
        if d==length(cf.c2D.detectors)
           % Remove all images from the subfolder (only once)
           for i=1:length(imageNames)
               delete([subfolder imageNames{i} '.jpg']);
           end 
        end
        
        detectorTime = toc;
        dl.Log(VerbosityLevel.Info,sprintf('Detector %s elapsed time: %.2f seconds.\n',D.name,detectorTime));
    end

end
