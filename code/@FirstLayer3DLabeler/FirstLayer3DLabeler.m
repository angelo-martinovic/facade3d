classdef FirstLayer3DLabeler < handle
    %FirstLayer3DLabeler Base class for semantic classification of 3D
    %point clouds.
    %   Detailed explanation goes here
    
    properties
        config = []; 
        sceneFull = [];  % Whole point cloud
        sceneTrain = []; % Training subset
        sceneTest = [];  % Testing subset

        
        Xtrain =[]; % Training data
        Ytrain =[];
        
        Xtest =[]; % Testing data
        
        prb = [];   % Resulting class probability per point
        cprb = [];  % Resulting labeling per point
        
    end
    %%
    methods (Access = public)
        % Constructor
        function fl = FirstLayer3DLabeler(datasetConfig)
            fl.config = datasetConfig;
            facade_init_scene(fl);
        end
        
        function PrepareData(obj)
            dl = DispatchingLogger.getInstance();
    
            dl.Log(VerbosityLevel.Info,' - Extracting PCL descriptors...\n');
            tic;
            obj.calc_features();
            elapsedTime = toc;
            dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',elapsedTime));
        end
        
        TrainClassifier(obj);
        
        RunClassifier(obj);
         
        PlotResults(obj);
         
        function pclLabeling = GetPCLLabeling(obj)
            cf = obj.config;
            c3d = cf.c3D;
 
            pclLabeling = get_adr('pcl_labeling',cf,c3d.name);
        end
        
        
    end
    %%
    methods(Access = private)
       
       [out1,out2,out3,out4] = facade_unary_class (obj,method,varargin);
       
       
       function facade_init_scene(obj)
            dl = DispatchingLogger.getInstance();

            % read data + descriptors
            %---- read basic data
            dl.Log(VerbosityLevel.Info,sprintf(' - read SScene_An.... dataset = ''%s'' \n',obj.config.name));

            obj.sceneFull = SScene_An();
            obj.sceneFull = obj.sceneFull.read_mat_data( obj.config );
            
            %--- separate into train/test
            obj.sceneTest = obj.sceneFull.keep_spec_data_ids(obj.sceneFull.flag==2);
            obj.sceneTrain  = obj.sceneFull.keep_spec_data_ids(obj.sceneFull.flag==1);
       end


       function calc_features(obj)
            dl = DispatchingLogger.getInstance();
            %---- calculate simple features from the entire point cloud
            obj.sceneFull = obj.sceneFull.calc_simple_features( obj.config );
            
            %--- separate into train/test
            obj.sceneTest = obj.sceneFull.keep_spec_data_ids(obj.sceneFull.flag==2);
            obj.sceneTrain  = obj.sceneFull.keep_spec_data_ids(obj.sceneFull.flag==1);
            
            % Remove unnecessary information
            obj.sceneFull.dist2plane=[];
            obj.sceneFull.nxyz=[];
            obj.sceneFull.heigh=[];
            obj.sceneFull.heigh_inv=[];
            obj.sceneFull.lindex=[];
            obj.sceneFull.oindex=[];
            obj.sceneFull.p_index=[];
            
            %--- calculate spin images, separate for train, test
            spinImageExtractorIdx = find(cellfun(@(x)strcmp(x.name,'spinImage'),obj.config.c3D.featureExtractors),1);
            if isempty(spinImageExtractorIdx)
                dl.Log(VerbosityLevel.Error,sprintf('Spin image extractor undefined! Check InitializeDataset.m and set datasetConfig.c3D to a 3D extractor'));
                error('Critical error. Terminating.');
            end
            spinImageExtractor = obj.config.c3D.featureExtractors{spinImageExtractorIdx};

            %--- read/calc desc
            dl.Log(VerbosityLevel.Info,sprintf(' - Extracting spin images from train set...\n'));
        %     tic;
            obj.sceneTrain = obj.sceneTrain.process_data(...
                'compute_get_si',...
                'split','train',...
                'si_dimensions',spinImageExtractor.si_dimensions,...
                'binSize',spinImageExtractor.binSize,...
                'imgW',spinImageExtractor.imgW,...
                'datasetConfig',obj.config);
        %     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));

        %     tic;
            dl.Log(VerbosityLevel.Info,sprintf(' - Extracting spin images from test set...\n'));
            obj.sceneTest = obj.sceneTest.process_data(...
                'compute_get_si',...
                'split','test',...
                'si_dimensions',spinImageExtractor.si_dimensions,...
                'binSize',spinImageExtractor.binSize,...
                'imgW',spinImageExtractor.imgW,...
                'datasetConfig',obj.config);
        %     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));

            %--- fix labeling ids
            obj.sceneTrain.lindex = obj.sceneTrain.lindex-1; %%% applies to monge where zero should be background...
            obj.sceneTest.lindex = obj.sceneTest.lindex-1; %%% applies to monge where zero should be background...

            dl.Log(VerbosityLevel.Info,sprintf(' - Concatenating descriptors...\n'));
            %--- concatenate descriptors to one vector
            [obj.sceneTrain, minDESC, maxDESC, V, D] = obj.sceneTrain.process_data('create_desc_from_weak_descs');

            %--- also desc
            obj.sceneTest = obj.sceneTest.process_data('create_desc_from_weak_descs','minX',minDESC,'maxX',maxDESC,'V',V,'D',D);
       
            % --- calculates the remaining features and prepares training matrices
            obj.facade_unary_class('get_desc');
       end

       
    end
    
end
