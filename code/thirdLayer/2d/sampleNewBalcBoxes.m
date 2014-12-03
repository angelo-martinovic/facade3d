function bbx = sampleNewBalcBoxes(origImg, balc_boundingBoxes, draw, dw, dh, maxnum)
    bbx= similarityVoting(origImg, balc_boundingBoxes', draw, dw, dh,3, maxnum);

end
