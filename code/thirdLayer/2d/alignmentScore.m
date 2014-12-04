function score = alignmentScore(bbs, bb, horrth, vertth)
    horscore = 1;
    vertscore = 1;
    for i=1:size(bbs,2)
       mt = max(bbs(2,i), bb(2));
       mb = min(bbs(4,i), bb(4));
       ml = max(bbs(1,i), bb(1));
       mr = min(bbs(3,i), bb(3));
        
       if (mb-mt)/(bb(4)-bb(2)) > 0.5
          horscore = horscore +1 ;
       end
       if (mr -ml)/(bb(3)-bb(1)) > 0.5
           vertscore = vertscore+1;
       end
       
    end
    %score = -log(1-1/vertscore) +(-log(1- 1/horscore));
    if vertscore >= horrth && horscore >=vertth
        score =1;
    else
        score = 0;
    end

end