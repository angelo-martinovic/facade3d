function [maxscore, maxpos]= symmetryDetect(im, symmetryRange,visualize)
        
    sift =0;
    harris =1;
   

    I = double(rgb2gray(im));

    
    if harris ==0
      if (sift ==0)
             [featureVectors,featureCoords] = getSSIMFeatNPos(I);

      else
             [featureVectors,featureCoords] = getSIFTFeatNPos(I);
      end
    else

%          sd = 0.7;
         si = 2.0;
sd = 0.001;
% si = 1.0;
         idx = vl_localmax( vl_harris( vl_imsmooth( I, sd ), si ) ) ;
         [i,j] = ind2sub( size(I), idx );

         C = [j;i];
        
        featureCoords = C';
    end
    if symmetryRange(1) ~=inf && symmetryRange(2) ~= inf
        
        featureCoords = featureCoords(featureCoords(:,2)>symmetryRange(1),:);
        featureCoords = featureCoords(featureCoords(:,2)<symmetryRange(2),:);
    end

    maxscore = -inf;
    maxpos = -1;
    from = size(im,2)/2;
    from = from - from/4;
    to = size(im,2)/2;
    to = to + to/4;
    from = round(from);
    to = round(to);
    scores = zeros(to-from+1,1);
    i=1;
    for x=from:to

       [score,n] = symmetryScore(featureCoords, x, size(im,2)/2,im,visualize);
       scores(i) = score;
       i=i+1;
       if score> maxscore
          maxscore = score;
          maxpos = x;
       end
        
    end
%     figure;plot(scores);

end
 









