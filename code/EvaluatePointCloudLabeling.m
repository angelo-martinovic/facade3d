function score = EvaluatePointCloudLabeling(labelingFilename)
    
    dl = DispatchingLogger.getInstance();
    cf = DatasetConfig.getInstance();
    
    dl.Log(VerbosityLevel.Info,sprintf(' - - Evaluating point cloud labeling %s.\n',labelingFilename));

    gtFilename = get_adr('pcl_gt_test');
    
    % Get labeling
    [points_full,~,colors_full]=ReadPCLFromPly(labelingFilename);
    labels_full = Colors2Labels(colors_full,cf.cm);

    % Get GT
    [pointsGT_full,~,colorsGT_full]=ReadPCLFromPly(gtFilename);

    labelsGT_full = Colors2Labels(colorsGT_full,cf.cm);

    assert(isequal(size(points_full),size(pointsGT_full)));
    assert(isequal(size(colorsGT_full),size(colors_full)));

    ignoreClasses = cf.ignoreClasses + 1;% necessary for Hayko's evaluation which counts from 1

    score = evaluation_multilabel(labelsGT_full,labels_full,ignoreClasses);

    % Save the output
    saveFilename = [labelingFilename '_results.mat'];
    save(saveFilename,'score');

    dl.Log(VerbosityLevel.Info,...
        sprintf(' - - Stored the result in %s. Pascal IoU score: %.2f.\n',...
        saveFilename,score.mean_pascal));
    
end