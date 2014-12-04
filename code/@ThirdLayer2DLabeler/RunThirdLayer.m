function RunThirdLayer(obj)

    facadeIDs = obj.GetFacadeIDs();    
    N = length(facadeIDs);

    if obj.condorEnabled
        condorScriptFilename = GenerateCondorScript_ThirdLayer(facadeIDs, obj.dirName, obj.baseName, obj.splitName);
        condorSubmitCmd = ['condor_submit ' condorScriptFilename];
        system(condorSubmitCmd,'-echo');
    else
        tic;
        pb = ProgressBar(N);
        for i=1:N   
            facadeID = facadeIDs(i);

            RunThirdLayerSingle(facadeID, obj.dirName, obj.modelName, obj.splitName,'local');
            pb.progress;
        end
        pb.stop;
        thirdLayerTime = toc;
        fprintf('Elapsed time: %d seconds.\n', thirdLayerTime);
    end
end
