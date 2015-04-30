classdef ConfigFactory < handle
    
    properties

    end
    %%
    methods (Static)
        function c = Make2DConfig(workFolder,nClasses,classifier,featureExtractor,detector,weights)
            c = c2D();
            

            %% Setup classifier
            if strcmp(classifier,'linearSVM')
                c.SetClassifier( classifier_linearSVM() );
                name = 'SVM';
                
            elseif strcmp(classifier,'nonlinearSVM')
                c.SetClassifier( classifier_nonlinearSVM() );
                name = 'SVM';
                
            else
                error('Unsupported classifier!');
            end
            
            %% Setup feature extraction
            if strcmp(featureExtractor,'gould')
                c.AddFeatureExtractor( featureExtractorGould(workFolder) );
                name = [name '_gould'];
                
            elseif strcmp(featureExtractor,'CNN')
                c.AddFeatureExtractor( featureExtractorCNN(workFolder) );
                name = [name '_CNN'];
                
            elseif strcmp(featureExtractor,'CNN+gould')
                c.AddFeatureExtractor( featureExtractorCNN(workFolder) );
                c.AddFeatureExtractor( featureExtractorGould(workFolder) );
                name = [name '_CNN_gould'];
                
            else
                error('Unsupported feature extractor!');
            end
            
            
            %% Setup detectors
            nDetectors = 0;
            if strcmp(detector,'none')
               % Do nothing
            elseif strcmp(detector, 'windowDetector')
                d1 = detector_window_generic();
                c.AddDetector(d1);
                name = [name '_winDet'];
                nDetectors = 1;
            else
                error('Unsupported detector!');
            end
                
            
            %% Setup CRF
            crf = crf2D();
            
            nExpected = 1+nDetectors+1;
            nReceived = length(weights);
            assert((nExpected==nReceived),...
                sprintf('Expected %d weights, received %d.\n',nExpected,nReceived));
            
            crf.weightUnarySegmentation = weights(1);
            crf.weightsUnaryDetectors = weights(2:end-1);
            crf.weightPairwise = weights(end);
            
            
            l = load('config/haussmannLabelCost.mat');
            crf.labelCost = l.labelCost(1:nClasses,1:nClasses); 
            
            c.SetCRF ( crf );
            c.SetName ( name );
        end
        
        function c = Make3DConfig(~,featureExtractor,detector,weights)
            c = c3D();
            
            %% Setup classifier
            % Currently only one supported: Random Forest
            c.SetClassifier ( classifier3D() );
            
            
            %% Setup feature extractors
            if strcmp(featureExtractor,'2D')
                name = '2D';
 
            elseif strcmp(featureExtractor,'spinImage')
                name = '3D';
                c.AddFeatureExtractor ( featureExtractorSpinImage() );
                
            elseif strcmp(featureExtractor,'spinImage+2D')
                name = '3D_2D';
                c.AddFeatureExtractor ( featureExtractorSpinImage() );
               
            else
                error('Unsupported feature extractor!');
            end
            
            %% Setup detectors
            nDetectors = 0;
            if strcmp(detector,'none')
                
            elseif strcmp(detector,'windowDetector')
                name = [name '_winDet'];
                nDetectors = 1;
            else
                error('Unsupported detector!');
            end

            
            %% Setup CRF
            crf = crf3D();
            
            nExpected = 1+1+nDetectors+1;
            nReceived = length(weights);
            assert((nExpected==nReceived),...
                sprintf('Expected %d weights, received %d.\n',nExpected,nReceived));
            
            crf.weight3DClassification = weights(1);
            crf.weight2DClassification = weights(2);
            crf.weights2DDetectors = weights(3:3+nDetectors-1);
            crf.weightPairwise = weights(end);
            
            c.SetCRF ( crf );
            c.SetName ( name );
        
        end
    end
end


            