classdef featureExtractorGould < featureExtractor2D
    %featureExtractorCNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

    end
    
    methods
        function F = featureExtractorGould(config)
            
            F@featureExtractor2D(config); 
            F.name = 'gould';
        end
        
        function ExtractFeatures(obj)
            dl = DispatchingLogger.getInstance();
            
            dl.Log(VerbosityLevel.Debug,sprintf(' - Gould feature extraction...\n'));
            workFolder = get_adr('work',obj.config);
            
            
            % Check if files exist
            calculatedFiles = dir([workFolder '*.features.' obj.name '.mat']);
            if isempty(calculatedFiles) || ~obj.config.useCache
            
                % Setup the external program
                openCVLib = 'external/lasik/external/opencv/lib/';

                featCmd = ['LD_LIBRARY_PATH=' openCVLib ' '...
                    'external/lasik/bin/segImageExtractFeatures -o ' workFolder ' '...
                    workFolder(1:end-1)];

                % Run the command
                [stat,res] = system(featCmd); %,'-echo');
                
                % Check that it finished succesfully
                if stat~=0
                     dl.Log(VerbosityLevel.Error,sprintf('Error running feature extraction: %s\n',res));
                     error('Critical error. Terminating.');
                end
                
                dl.Log(VerbosityLevel.Debug,sprintf(' - Gould feature extraction done. Creating mat files...\n'));
                
                % Create the mat files from txt files
                calculatedFiles = dir([workFolder '*.features.txt']);
                for i=1:length(calculatedFiles);
                    % Read the generated text file
                    features = dlmread([workFolder calculatedFiles(i).name]); %#ok<NASGU>
                    featName = get_adr('2D_features',obj.config,calculatedFiles(i).name(1:end-13),obj.name);
                    
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

