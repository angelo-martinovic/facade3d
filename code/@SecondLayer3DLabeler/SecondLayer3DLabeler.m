classdef SecondLayer3DLabeler < handle
    %SecondLayer3DLabeler 
    %   Detailed explanation goes here
    
    properties
       
        scene = [];
%         idxs = [];
        unaries = [];
        
        labelsMAP =[];
        labelsCRF = [];
        unary = [];
    end

    methods (Access = public)
        % Constructor
        function sl = SecondLayer3DLabeler(scene)
            initialize(sl,scene);
        end
        
        Run3DCRF(obj);
    
        SavePotentialsAndLabels(obj)
        
        function outputPCLName = GetOutputNameMAP(obj)
            outputPCLName = [getOutputName(obj) '_3DMAP'];
        end
        
        function outputPCLName = GetOutputNameCRF(obj)
            outputPCLName = [getOutputName(obj) '_3DCRF'];
        end
      
    end

    methods(Access = private)
       initialize(obj,scene);
        
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
