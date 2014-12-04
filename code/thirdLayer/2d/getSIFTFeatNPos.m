function  [featureVectors,featureCoords] =getSIFTFeatNPos(I)


 [C, featureVectors] = vl_sift(single(I));

  featureCoords= C(1:2,:)';
  featureVectors = featureVectors';

end













