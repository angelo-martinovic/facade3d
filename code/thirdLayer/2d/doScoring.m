function score = doScoring(boxes, outImg, hyperParameters)

    score = hyperParameters.gridweight * scoreWindowGrid(boxes, hyperParameters) + ...
            hyperParameters.dataweight * scoreDataTerm2(outImg, boxes, hyperParameters);
        
    if hyperParameters.principles.cooccurence
        score = score +   hyperParameters.coOccWeight * scoreCoOccurence(boxes,hyperParameters);
    end


end