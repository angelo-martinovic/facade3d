function submitted = RunThirdLayer(obj)
    dl = DispatchingLogger.getInstance();
    dl.Log(VerbosityLevel.Info,sprintf('Running the third layer...\n'));
    
    facadeIDs = obj.GetFacadeIDs();    
    N = length(facadeIDs);

    datasetConfig = obj.config; %#ok<NASGU>
    save('datasetConfig.mat','datasetConfig');
    
    submitted = false;
    if obj.condorEnabled
        condorScriptFilename = GenerateCondorScript_ThirdLayer( ...
            facadeIDs, ...
            obj.config.name, ...
            obj.splitName);
        condorSubmitCmd = ['condor_submit ' condorScriptFilename];
        system(condorSubmitCmd,'-echo');
        submitted = true;
    else
        tic;
        pb = ProgressBar(N);
        for i=1:N   
            facadeID = facadeIDs(i);
            RunThirdLayerSingle(obj.splitName,facadeID,'local');
            pb.progress;
        end
        pb.stop;
        dl.Log(VerbosityLevel.Info,sprintf('Done. Elapsed time: %.2f seconds.\n',toc));
    end
end
