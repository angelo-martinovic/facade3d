classdef ScreenLogger < ILogger
    %ScreenLogger Shows the message on the screen.
    %   Detailed explanation goes here
    
    properties
        verbosityLevel
    end
    
    methods
        function obj = ScreenLogger(verbosity)
            assert(isa(verbosity,'VerbosityLevel'),...
                'Wrong format for verbosity. Use the enum VerbosityLevel!');
            
            obj.verbosityLevel = verbosity;
        end
        
        function Log(obj,verbosity,message)
            assert(isa(verbosity,'VerbosityLevel'),...
                'Wrong format for severity. Use the enum VerbosityLevel!');
            if uint32(verbosity)<=(obj.verbosityLevel)
                typeChar = char(verbosity);
                
                fprintf(['[' typeChar(1)  '] ' message]); 
            end
        end
    end
    
end

