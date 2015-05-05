classdef featureExtractorGould < featureExtractor2D
    %featureExtractorGould Summary of this class goes here
    %   Detailed explanation goes here
    properties
        overwriteExistingFiles = false;
        
    end
    
    methods
        function F = featureExtractorGould(workFolder)
            
            F@featureExtractor2D(workFolder); 
            F.name = 'gould';
        end
        
        function ExtractFeatures(obj)
            dl = DispatchingLogger.getInstance();
            
            dl.Log(VerbosityLevel.Debug,...
                sprintf(' - Gould feature extraction...\n'));
            workFolder = obj.workFolder;
            
            % Check if files exist
            calculatedFiles = dir([workFolder '*.features.' obj.name '.mat']);
            if isempty(calculatedFiles) || obj.overwriteExistingFiles
            
                % Setup the external program
                openCVLib = 'external/lasik/external/opencv/lib/';

                featCmd = ['LD_LIBRARY_PATH=' openCVLib ' '...
                    'external/lasik/bin/segImageExtractFeatures -o ' workFolder ' '...
                    workFolder(1:end-1)];

                % Run the command
                [stat,res] = system(featCmd); %,'-echo');
                
                % Check that it finished succesfully
                if stat~=0
                     ME = MException('FeatureExtractorGould:segImageExtractFeaturesFailed', ...
                           'Error running feature extraction: %s\n',res);
                     throw(ME);
                end
                
                dl.Log(VerbosityLevel.Debug,...
                    sprintf(' - Gould feature extraction done. Creating .mat files...\n'));
                
                % Create the mat files from txt files
                calculatedFiles = dir([workFolder '*.features.txt']);
                for i=1:length(calculatedFiles);
                    % Read the generated text file
                    features = dlmread([workFolder calculatedFiles(i).name]); %#ok<NASGU>
                    featName = [workFolder calculatedFiles(i).name(1:end-13) '.features.' obj.name '.mat'];
                    
                    % Save features as a mat file
                    save(featName,'features');
                    
                    % Delete the txt file
                    delete([workFolder calculatedFiles(i).name]);
                end
                dl.Log(VerbosityLevel.Debug,sprintf(' - Done.\n'));
            else
                dl.Log(VerbosityLevel.Debug,sprintf(' - Found precalculated features. Skipping.\n'));
            end   
        end
    end
end
