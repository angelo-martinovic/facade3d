classdef SecondLayer3DLabeler < handle
    %SecondLayer3DLabeler 
    %   Detailed explanation goes here
    
    properties
        config = [];
        
        pointsFull = [];
        idxs = [];
        unaries = [];
        
        labelsMAP =[];
        labelsCRF = [];
        unary = [];
    end

    methods (Access = public)
        % Constructor
        function sl = SecondLayer3DLabeler(datasetConfig)
            sl.config = datasetConfig;
            initialize(sl);
        end
        
        LabelPointCloudWithUnaries(obj);
    
        SavePotentialsAndLabels(obj)
        
        function outputPCLName = GetOutputNameCRF(obj)
            outputPCLName = [getOutputName(obj) '_3DCRF'];
        end
      
    end

    methods(Access = private)
       initialize(obj);
        
       function outputPCLName = getOutputName(obj)
    
            outputPCLName = '';
            connector = '';
            for i=1:length(obj.unaries)
                if obj.unaries(i).weight>0
                    outputPCLName = [outputPCLName connector obj.unaries(i).name];    %#ok<AGROW>

                    if strcmp(connector,'')
                        connector='+';
                    end
                end
            end

       end
    
    end
    
end
