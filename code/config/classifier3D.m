classdef classifier3D < handle
    %CLASSIFIER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        name = 'rf';
        desc_reco      = 'desc'; %%% which descriptor, desc=concatenation of all descs.  
        unary_classifier = 'rf'; %'liblinear';%   %%classifier, rf=random forest, liblinear=linear SVM
        sample     = 0;%4e3;%0;%2e5;   %%% # of samples per class
        norm_data = 0;  % Normalize data
        
        % RF parameters
        nrf_tree   = 100;   %%% # of trees in RF
        min_leaf  = 30;
        oob_pred = 'On'; 
        cost_matrix = [0 1 1 1 1 1 1; ...  % Unused
                       1 0 1 1 1 1 1; ...
                       1 1 0 1 1 1 1; ...
                       2 2 2 0 2 2 2; ...
                       1 1 1 1 0 1 1 ;...
                       1 1 1 1 1 0 1 ;...
                       1 1 1 1 1 1 0 ];
    end
    
    methods (Access = public)
        function SetClassifierName(obj,val)
            obj.name = val;
        end
        
        function SetMinLeaf(obj,val)
            obj.min_leaf = val;
        end
        
        function SetNrfTree(obj,val)
            obj.nrf_tree = val;
        end
    end
        
    
end

