classdef FirstLayer3DLabeler < handle
    %FirstLayer3DLabeler Base class for semantic classification of 3D
    %point clouds.
    %   Detailed explanation goes here
    
    properties
        config = [];   
        test_data = [];
        train_data = [];
    end
    %%
    methods (Access = public)
        % Constructor
        function fl = FirstLayer3DLabeler(datasetConfig)
            fl.config = datasetConfig;
        end
        
        facade_init_all_data(obj);
        
        [out1,out2,out3,out4] = facade_unary_class (obj,method,varargin);
      
    end
    %%
    methods(Access = private)
       
        
    end
    
end
