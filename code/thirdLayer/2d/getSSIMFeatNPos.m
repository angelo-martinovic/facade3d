function [featureVectors featureCoords] = getSSIMFeatNPos(I)
    sd = 0.7;
    si = 2.0;
    idx = vl_localmax( vl_harris( vl_imsmooth( I, sd ), si ) ) ;
    [i,j] = ind2sub( size(I), idx );
    %[C,resp] = vl_sift(single(I));
    C = [j;i]';
    %setup ssim features
    parms.size=5;
    parms.coRelWindowRadius=20;
    parms.numRadiiIntervals=8;
    parms.numThetaIntervals=15;
    parms.varNoise=25*3*36;
    parms.autoVarRadius=1;
    parms.saliencyThresh=0; % I usually disable saliency checking
    parms.nChannels=size(I,3);
    radius=(parms.size-1)/2; % the radius of the patch
    marg=radius+parms.coRelWindowRadius;




    %check bounds
    [allX, allY] = remPclose2Border(C(:,1),C(:,2),size(I,2)-marg,marg+1, size(I,1)-marg,marg+1);
    
    [featureVectors,featureCoords,salientCoords,uniformCoords]=ssimDescriptor(I ,parms ,allX' ,allY');
    featureVectors = featureVectors';
    featureCoords = featureCoords';
end