function hyperParameters = SetupHyperParameters(profileName)

    hyperParameters = SetupHyperParametersDefault;

    if strcmp(profileName,'local')
        hyperParameters = SetupHyperParametersLocal(hyperParameters);
    elseif strcmp(profileName,'condor')
        hyperParameters = SetupHyperParametersCondor(hyperParameters);
    else
        error('Unknown hyper-parameters profile!');
    end

end


function hyperParameters = SetupHyperParametersDefault()
    hyperParameters = struct();

    hyperParameters.legacy = false; % ECCV code
    
    hyperParameters.parallel = true;
    hyperParameters.visualize = false;
%     hyperParameters.rectification = false;

    hyperParameters.parMinObjectSize = 200;
    hyperParameters.parMinWidth = 10;
    hyperParameters.parMinHeight = 10;
    
    hyperParameters.parMaxMedWinSize = 50;
                                      % ECCV params
    hyperParameters.dataweight = 1;   %80; 
    hyperParameters.gridweight = 1;   %5;
    hyperParameters.coOccWeight = 1;  %10;

%     hyperParameters.ga.nGenerations = 300;
%     hyperParameters.ga.stallGenLimit = 30;
%     hyperParameters.ga.populationSize = 1000;

    hyperParameters.objClasses = [1 3 4];
    hyperParameters.winClass = 1; 
    hyperParameters.balcClass = 3; 
    hyperParameters.doorClass = 4;

    hyperParameters.principles.alignment = true;
    hyperParameters.principles.similarity = false;
    hyperParameters.principles.symmetry = true;
    hyperParameters.principles.verticalRegionOrder = true;
    hyperParameters.principles.door = true;
    hyperParameters.principles.cooccurence = true;

    hyperParameters.overrideClasses = []; % classes that should not be overlaid

%     hyperParameters.t_sym = 0.5;    % symmetry threshold
%     hyperParameters.colormap = 'haussmann';

end

function hyperParameters = SetupHyperParametersLocal(hyperParameters)
    hyperParameters.parallel = false;
    hyperParameters.visualize = true;
%     hyperParameters.ga.populationSize = 10;
end

function hyperParameters = SetupHyperParametersCondor(hyperParameters)
    hyperParameters.parallel = true;
    hyperParameters.visualize = false;
%     hyperParameters.ga.populationSize = 1000;

end