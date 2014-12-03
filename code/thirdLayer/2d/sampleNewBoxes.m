function bbx = sampleNewBoxes(origImg, win_boundingBoxes, draw, dw, dh, maxnum)


   %use similarity voting
   bbx= similarityVoting(origImg, win_boundingBoxes', draw, dw, dh,1, maxnum);
   
   %use symmetry
   %[symscore, sympos]= symmetryDetect(origImg, win_boundingBoxes',[ypos(2) ypos(4)])
%    if symscore > 0.5
%         win_boundingBoxes = updateMapsbySymmetryAndBB(win_boundingBoxes, sgmp,sympos, 1,win_ddw, win_ddh,1);
%         %balc_boundingBoxes = updateMapsbySymmetryAndBB(balc_boundingBoxes, sgmp,sympos, 3,balc_ddw, balc_ddh,0);     
% 
%    
%    end
   
   %newmax = findnewMaxima(vs,win_boundingBoxes,maxvs);
   %evaluate new maxima
%    score = zeros(1,size(newmax,2));
%    refscore = objfun3(win_boundingBoxes,PM_win,win_ddw, win_ddh);
%    for i=1:size(newmax,2)
%        winbbtmp =  [win_boundingBoxes newmax(:,i)];
%        % winbbtmp = fminunc(@(wdrawRects(inbbtmp) objfun3(winbbtmp,PM_win,win_ddw, win_ddh), winbbtmp);
%        score(i) =  alignmentScore(win_boundingBoxes, newmax(:,i),3,3 )%    objfun3(winbbtmp,PM_win,win_ddw, win_ddh)
%        %drawRects(winbbtmp, 988, origImg);
%        figure(112); imagesc(origImg);
%        
%        
%    end
%    
%    
%    
%    win_boundingBoxes = [win_boundingBoxes  newmax(:,(score==1))];


end