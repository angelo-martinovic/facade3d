function score = EvaluateMeshLabeling(labelingFilename,gtFilename)
    
    cm = HaussmannColormap;
    
    % Get labeling
    [points_full,~,colors_full]=ReadPCLFromPly(labelingFilename);
    labels_full = Colors2Labels(colors_full,cm);
    
    % Get GT
    [pointsGT_full,~,colorsGT_full]=ReadPCLFromPly(gtFilename);
    
    labelsGT_full = Colors2Labels(colorsGT_full,cm);
    
    assert(isequal(size(points_full),size(pointsGT_full)));
    assert(isequal(size(colorsGT_full),size(colors_full)));

    ignoreClasses = [1 9];
    
    score = evaluation_multilabel(labelsGT_full,labels_full,ignoreClasses);

    
end