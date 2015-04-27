classdef ThirdLayer3DLabeler
    %THIRDLAYER3DLABELER Summary of this class goes here
    %   Detailed explanation goes here
    
   properties
        config = [];
        splitName = [];
        
        pcl_test = [];
        pcl_all = [];
    end

    methods (Access = public)
        % Constructor
        function sl = ThirdLayer3DLabeler(datasetConfig,modelName,pcl_test,pcl_all)
            sl.config = datasetConfig;
            sl.splitName = modelName;
            sl.pcl_test = pcl_test;
            sl.pcl_all = pcl_all;
        end
        
        RunThirdLayer(obj);
        
        function outputPCLName = GetOutputName(obj)
             outputPCLName = get_adr('3D_L3_Pure3D_labeling',obj.config,obj.splitName);
        end
      
    end

    methods(Access = private)
      
    
    end
    
end

