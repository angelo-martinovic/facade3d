% Gets all images, rectifies, segments, extracts features, runs detectors on test set.
function PrepareData(obj)
    %% 1st layer - prepare data


    % Source folder
    dirName = obj.dirName;

    % Create work folder
    workFolder = [dirName 'work/'];
    if ~exist(workFolder,'dir')
        mkdir(workFolder);
    end

    %% Get the list of all images
    subset = 'all';
    file_str_idx = obj.LoadFilenames(subset);

    imageNames = strcat(file_str_idx,'.jpg');%cellfun(@(x)strcat(x,'.jpg'),file_str_idx,'UniformOutput',false);
    imageFilenames = strcat([dirName 'image/'],imageNames);%cellfun(@(x)strcat(,x),imageNames,'UniformOutput',false);

    %% Run rectification
    tic;
    fprintf('Rectifying images ...');
%     pb = ProgressBar(length(imageFilenames));
    parfor i=1:length(imageFilenames)
        rectCmd = ['LD_LIBRARY_PATH= python rectification/rectifyImage.py -t auto -i ' imageFilenames{i} ' -o ' workFolder imageNames{i}];

        [stat,res] = system(rectCmd);
        if stat~=0
            warning('Rectification failed on image %d with message %s\n',i,res);
            continue;
        end

%         pb.progress;
    end
%     pb.stop;
    rectificationTime = toc;
    fprintf('Elapsed time: %d seconds.\n',rectificationTime);

    %% Remove empty images
    cntRemoved = 0;
    for i=1:length(imageFilenames)
        s = dir([workFolder imageNames{i}]);
        if isempty(s)
            delete([workFolder imageNames{i}]);
            delete([workFolder imageNames{i}(1:end-4) '_rect.dat']);
            delete([workFolder imageNames{i}(1:end-4) '_scale.dat']);
            cntRemoved = cntRemoved+1;
            continue;
        end
        if s.bytes==0
            delete([workFolder imageNames{i}]);
            delete([workFolder imageNames{i}(1:end-4) '_rect.dat']);
            delete([workFolder imageNames{i}(1:end-4) '_scale.dat']);
            cntRemoved = cntRemoved+1;
            continue;
        end
    end
    fprintf('Removed %d empty images.\n',cntRemoved);

    %% Resize images
    tic;
    fprintf('Resizing images to common height: %d ...',obj.resizedImagesHeight);
    resizeCmd = ['mogrify -resize x' num2str(obj.resizedImagesHeight) ' ' workFolder '*.jpg'];
    [stat,res] = system(resizeCmd); %,'-echo');
    if stat~=0
        error('Error running mogrify resize: %s\n',res);
    end
    resizeTime = toc;
    fprintf('Elapsed time: %d seconds.\n',resizeTime);

    %% Convert to ppm
    tic;
    fprintf('Converting to ppm (needed for meanshift) ...');
    convertCmd = ['mogrify -format ppm ' workFolder '*.jpg'];
    [stat,res] = system(convertCmd); %,'-echo');
    if stat~=0
        error('Error running mogrify convert: %s\n',res);
    end
    convertTime = toc;
    fprintf('Elapsed time: %d seconds.\n',convertTime);

    %% Run meanshift
    tic;
    fprintf('Meanshift segmentation ...');
    parfor i=1:length(imageFilenames)
        if exist([workFolder imageNames{i}(1:end-4) '.ppm'],'file')
            copyfile([workFolder imageNames{i}(1:end-4) '.ppm'],[obj.edisonLoc 'tmp/']);
            segCmd = [obj.edisonLoc 'segmentImage.sh ' imageNames{i}(1:end-4) '.ppm' ' ' num2str(obj.minRegionArea)];
            [stat,res] = system(segCmd); %,'-echo');
            if stat~=0
                error('Error running segmentImage: %s\n',res);
            end
            delete([obj.edisonLoc 'tmp/' imageNames{i}(1:end-4) '.ppm']);
            movefile([obj.edisonLoc 'tmp/' imageNames{i}(1:end-4) '.seg'],workFolder);
        end
    end
    segmentationTime = toc;
    fprintf('Elapsed time: %d seconds.\n',segmentationTime);

    %% Run gould feature extraction
    tic;
    fprintf('Feature extraction ...');
    featCmd = ['LD_LIBRARY_PATH=' obj.lasicLoc 'external/opencv/lib/ ' obj.lasicLoc 'bin/segImageExtractFeatures -o ' workFolder ' ' workFolder(1:end-1)];
    [stat,res] = system(featCmd); %,'-echo');
    if stat~=0
        error('Error running feature extraction: %s\n',res);
    end
    featureExtractionTime = toc;
    fprintf('Elapsed time: %d seconds.\n',featureExtractionTime);
   
    %% Run detector(s)
    for d=1:length(obj.detectors)
        detectorName = obj.detectors{d}.name;

        fprintf('Detector %s ...',detectorName);
        windowDetectorConfigFile = obj.detectors{d}.configFile;
        windowDetectorModelFile = obj.detectors{d}.modelFile;

        subfolder = [workFolder 'detections/'];
        if ~exist(subfolder,'dir')
            mkdir(subfolder);
%             while ~exist(subfolder,'dir')
%                 pause(0.1);
%             end
        else
            system(['rm -r ' subfolder]);%,'-echo');
            mkdir(subfolder);
        end

        % Copy test images to a subfolder
        testImages = obj.LoadFilenames('eval');
        testImages = strcat(testImages,'.jpg');

        for i=1:length(testImages)
             if exist([workFolder testImages{i}],'file')
                copyfile([workFolder testImages{i}],subfolder);
             end
        end

        % Run detector on the subfolder
        tic;
        detCmd = [obj.biclopLoc 'src/applications/objects_detection/objects_detection -c ' ...
                    windowDetectorConfigFile ' --objects_detector.model ' windowDetectorModelFile ...
                    ' --recording_path '  subfolder  ' --process_folder '  subfolder ];
        [stat,res] = system(detCmd);   %,'-echo');
        if stat~=0
            error('Error running object detector: %s\n',res);
        end

        detCmd2 = ['python ' obj.biclopLoc '/tools/objects_detection/detections_to_wnd_eval_normalized.py -d ' subfolder ...
            'detections.data_sequence -o ' subfolder 'detections-' detectorName ' -m ' windowDetectorModelFile];
        [stat,res] = system(detCmd2);%,'-echo');
        if stat~=0
            error('Error creating detection files: %s\n',res);
        end
        delete([subfolder 'detections.data_sequence']); 
    end
    detectorTime = toc;
    fprintf('Elapsed time: %d seconds.\n',detectorTime);
end

