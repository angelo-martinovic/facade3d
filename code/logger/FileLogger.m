classdef FileLogger < ILogger
    %FileLogger Saves the message in a log file.
    %   Detailed explanation goes here
    
    properties
        verbosityLevel
        filePath
    end
    
    methods
        function obj = FileLogger(verbosity,path)
            if nargin~=2
                error('Wrong number of arguments! Use FileLogger(verbosity,path).');
            end
            
            assert(isa(verbosity,'VerbosityLevel'),...
                'Wrong format for verbosity. Use the enum VerbosityLevel!');
            
            obj.verbosityLevel = verbosity;
            
            if ~exist(path,'file')
                warning('Logfile does not exist. Attempting to create it.');
                try
                    fclose(fopen(path, 'w'));
                catch err
                   error('Invalid log file.\nLog file creation failed with message:\n "%s" \n',err.message); 
                end
            end
            obj.filePath = path;
        end
        
        function Log(obj,verbosity,message)

            assert(isa(verbosity,'VerbosityLevel'),...
                'Wrong format for severity. Use the enum VerbosityLevel!');
            
            if uint32(verbosity)<=(obj.verbosityLevel)
                typeChar = char(verbosity);
                
                
                if ~strendswith(message,'\n') && ~strendswith(message,char(10))
                    message = [message '\n'];
                end
                message = ['[' datestr(now) ']' '[' typeChar(1) '] ' message];
                
                
                f = fopen(obj.filePath,'a');
                fprintf(f, message); 
                fclose(f);
                
                
%                 [stat,res]=system(['flock -x ' obj.filePath ' -c '' echo -e "' message '" >> ' obj.filePath ' '' ']);
            end
        end
    end
    
end

