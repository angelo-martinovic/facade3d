classdef FirstLayer3DLabeler < handle
    %FirstLayer3DLabeler Base class for semantic classification of 3D
    %point clouds.
    %   Detailed explanation goes here
    
    properties
        config = [];   
        test_data = [];
        train_data = [];
        
        Xtrain =[];
        Ytrain =[];
        Xtest =[];
        
        prb = [];
        cprb = [];
        
    end
    %%
    methods (Access = public)
        % Constructor
        function fl = FirstLayer3DLabeler(datasetConfig)
            fl.config = datasetConfig;
            facade_init_all_data(fl);
        end
        
        PrepareData(obj);
        
        TrainClassifier(obj);
        
        EvaluateClassifier(obj);
         
        PlotResults(obj);
         
        function pclLabeling = GetPCLLabeling(obj)
            cf = obj.config;
            c = cf.c3D.classifier;
 
            pclLabeling = get_adr('pcl_labeling',cf,c.name);
        end
        
        
    end
    %%
    methods(Access = private)
       facade_init_all_data(obj);
       [out1,out2,out3,out4] = facade_unary_class (obj,method,varargin);
    end
    
end