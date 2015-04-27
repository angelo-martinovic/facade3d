classdef detector_window_generic < detector
    %DETECTOR_WINDOW_GENERIC Summary of this class goes here
    %   Detailed explanation goes here
    
properties
        configFile 
        modelFile 
        
        meanDetection
end

methods
    function D = detector_window_generic(config)
        D.name = 'window-generic';
        D.class = 1;
        D.configFile = 'detector/configs/window-generic.ini';
        D.modelFile = 'detector/models/window-generic.bin';
        
        D.meanDetection = get_adr('2D_detectorMeanDetectionTrain', config, D.name);
    end

    function DetectObjects(obj,subfolder,outputFolder)
        dl = DispatchingLogger.getInstance();
 
        detCmd = ['LD_LIBRARY_PATH="" && '...
               'external/biclop/src/applications/objects_detection/objects_detection -c ' ...
               obj.configFile ' --objects_detector.model ' obj.modelFile ...
               ' --recording_path '  subfolder  ' --process_folder '  subfolder ];
        [stat,res] = system(detCmd,'-echo'); % add for debug
        if stat~=0
           ME = MException('WindowDetector:detectionFailed', ...
                          'Error running object detector: %s\n',res);
           throw(ME);
        end


        dl.Log(VerbosityLevel.Debug,sprintf('Creating detection files ...\n'));
        detCmd2 = ['python external/biclop/tools/objects_detection/'...
                   'detections_to_wnd_eval_normalized.py'...
                   ' -d ' subfolder 'detections.data_sequence' ...
                   ' -o ' outputFolder ...
                   ' -m ' obj.modelFile];
        [stat,res] = system(detCmd2,'-echo'); % add for debug
        if stat~=0
            ME = MException('WindowDetector:dataSequenceFailed', ...
                           'Error creating detection files: %s\n',res);
            throw(ME);
        end
        delete([subfolder 'detections.data_sequence']); 
    end
    
    function CalculateMeanDetection(obj,config,imageNames)
%         dl = DispatchingLogger.getInstance();
        
        
        meanDet = zeros(100,100,config.nClasses);
        totalNDet = 0;
        for i=1:length(imageNames)
            detectionFilename = get_adr('2D_detectionsTrain',config,obj.name,imageNames{i});
            gtFilename = get_adr('2D_label',config,imageNames{i});
            
            dets = detector_window_generic.readDetections(detectionFilename);
            gt = imread(gtFilename);
            groundTruth = Image2Labels(double(gt), config.cm);     

            % Rectify ground truth
            rectParamsFilename = get_adr('2D_rectParams',config,imageNames{i});
            if exist(rectParamsFilename,'file')
                h = dlmread(rectParamsFilename);
                h = reshape(inv(reshape(h,3,3)),1,9);
                groundTruth = rewarp(target,groundTruth,h);
                groundTruth = imresize(groundTruth,size(segmentation),'nearest');
            end
        
            height = size(groundTruth,1);
            width = size(groundTruth,2);
            numDetections = size(dets,1);

            if numDetections>0
                % Map detection bounding boxes to unary potentials
                for d = 1:numDetections
                    detectionRect = dets(d,1:4);  % Detection bounding box rectangle
%                     detectionScore = dets(d,5);   % Score of the detection

                    startRow = max(round(detectionRect(2)),1);
                    endRow = min(round(detectionRect(4)),height);
                    startColumn = max(round(detectionRect(1)),1);
                    endColumn = min(round(detectionRect(3)),width);


                    if (endRow-startRow<1) || (endColumn-startColumn<1)
                        continue;
                    end

                    gtDet = groundTruth(startRow:endRow,startColumn:endColumn);
                    for c=1:config.nClasses
                        meanDet(:,:,c) = meanDet(:,:,c) + imresize((gtDet==c),[100 100],'nearest');
                    end
                    totalNDet = totalNDet+1;
                    
                end
            end
                
            
        end
        meanDet=bsxfun(@rdivide,meanDet,sum(meanDet,3));
        meanDet(isnan(meanDet))=1/config.nClasses; %#ok<NASGU>
        
        save(obj.meanDetection,'meanDet');
    end
    
   
    
    function detectionMap = ProbabilityMapFromDetections(obj,detectionFilename,nClasses,height,width)
        
        dets = detector_window_generic.readDetections(detectionFilename);
        
        
        load(obj.meanDetection); % loads meanDet
        meanDet = meanDet(:,:,1:nClasses); %#ok<NODEF>
        detectionMap = 1/nClasses* ones(height,width,nClasses);

        numDetections = size(dets,1);

        if numDetections>0
%             rangeScores = [min(dets(:,5)) max(dets(:,5))];
%             rangeProbs = [1/nClasses (1-0.001*(nClasses-1))];
            
            % Map detection bounding boxes to unary potentials
            for d = 1:numDetections
                detectionRect = dets(d,1:4);  % Detection bounding box rectangle
%                 detectionScore = dets(d,5);   % Score of the detection

                % Validation set-based mapping
                % [~,pos] =min(abs([labelMaps.score]-detectionScore));

                % Linear mapping
%                 relScore = (detectionScore-min(rangeScores))/(max(rangeScores)-min(rangeScores));
%                 if isnan(relScore)
%                     relScore = min(rangeScores);
%                 end
%                 classProb = min(rangeProbs) + relScore* (max(rangeProbs)-min(rangeProbs));

                % Max mapping
                % classProb = max(rangeProbs);

%                 nonClassProb = (1-classProb)/(nClasses-1);
%                 assert(~isnan(nonClassProb),'NaN detector class probability!');
                startRow = max(round(detectionRect(2)),1);
                endRow = min(round(detectionRect(4)),height);
                startColumn = max(round(detectionRect(1)),1);
                endColumn = min(round(detectionRect(3)),width);

%                 detHeight = endRow-startRow+1;
%                 detWidth = endColumn-startColumn+1;

                if (endRow-startRow<1) || (endColumn-startColumn<1)
                 continue;
                end

%                 probMap = zeros(detHeight,detWidth,nClasses);
%                 for c=1:nClasses
%                     if c~=detector.class
%                         probMap(:,:,c) = nonClassProb * ones(detHeight,detWidth);
%                     else
%                         probMap(:,:,c) = classProb * ones(detHeight,detWidth);
%                     end
%                 end

                probMap = imresize(meanDet,[endRow-startRow+1 endColumn-startColumn+1],'nearest');
                detectionMap(startRow:endRow,startColumn:endColumn,:) = probMap;
            end   
        end
    end
    
     
    
end

 methods(Static)
      
    function dets = readDetections(detectionFilename)
         if ~isempty(detectionFilename)
            s = dir(detectionFilename);
            if isempty(s)
                dets = [];
            else
                if s.bytes == 0
    %                 dl.Log(VerbosityLevel.Warning,sprintf('No %s detections found in this image.',detector.name));
                    dets =[];
                else
                    dets = dlmread(detectionFilename);
                end;   
            end
        else
            dets =[];
        end
    end
end
    
end

