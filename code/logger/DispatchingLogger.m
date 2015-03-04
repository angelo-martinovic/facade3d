classdef (Sealed) DispatchingLogger < ILogger
    %DispatchingLogger Dispatches the message to all subscribed loggers.
    %   Singleton class.
    
    properties (SetAccess = private)
        loggers  = {};
    end
    
    methods (Access = private)
      function obj = DispatchingLogger
      end
    end
    
    methods (Access = public)
        function Log(obj,verbosity,message)
            for i=1:length(obj.loggers)
               obj.loggers{i}.Log(verbosity,message);
            end
        end
        
        function Subscribe(obj,logger)
            obj.loggers{end+1} = logger;
        end
        
        function Clear(obj)
            obj.loggers = {};
        end
    end
    
   methods (Static)
      function singleObj = getInstance
         persistent localObj
         if isempty(localObj) || ~isvalid(localObj)
            localObj = DispatchingLogger;
         end
         singleObj = localObj;
      end
   end
    
end
