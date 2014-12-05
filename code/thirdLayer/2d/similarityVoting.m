function bbx = similarityVoting(im,bbs, draw, dw, dh, cls, maxnum)
    addpath '/esat/sadr/amartino/Code/vlfeat-0.9.19/toolbox/';
    addpath '/esat/sadr/amartino/Code/ssim/';
    
    vl_setup;
    
    old_bbs = bbs;
    %make the boxes bigger to contain window surroundings
    for i=1:size(bbs,1)
        bbs(i,:) = increasesize(bbs(i,:),1.3);
    end
    

    sift =0;

    I = double(rgb2gray(im));
    vs = zeros(size(I));

    if (sift ==0)
        [featureVectors,featureCoords] = getSSIMFeatNPos(I);

    else
         [featureVectors,featureCoords] = getSIFTFeatNPos(I);
    end


    %split features
    [vs, wss, hss] = performVoting(vs, bbs, old_bbs, featureVectors, featureCoords, draw, dw, dh);
    
    maxvs = max(vs(:));

    bbs = round(bbs);
    bbs = setBBtoImage(bbs',size(im,2), size(im,1));
    p2 = bbs;
    minvs = max(maxvs);
    for i=1:size(p2,2)
        maxInBB = max(max(vs((p2(2,i)):p2(4,i),p2(1,i):p2(3,i))));
        minvs = min(minvs,maxInBB);
        %vs((p2(2,i)):p2(4,i),p2(1,i):p2(3,i)) = 0;
        %allMax = [allMax maxInBB];
    end
    vs = myMaxFilter( vs, median(bbs(3,:) - bbs(1,:)), median(bbs(4,:) - bbs(2,:)));
    %minvs = max(maxvs);
    %setting vs to zero where we have already detections
    %allMax = []
    for i=1:size(p2,2)
        maxInBB = max(max(vs((p2(2,i)):p2(4,i),p2(1,i):p2(3,i))));
        %minvs = min(minvs,maxInBB)
        vs((p2(2,i)):p2(4,i),p2(1,i):p2(3,i)) = 0;
        %allMax = [allMax maxInBB];
    end
%     allMax = sort(allMax,'descend')
%     if numel(allMax > size(bbs,2))
%         minvs = allMax(size(bbs,2));
%     end
    %vs(vs<=minvs) =0;
    bbx = [];
    c = 0;
    while c +size(bbs,2)< maxnum
        c = c+1;
        %[x1,y1, x2,y2, classidx, score];
       [mv,idx] = max(vs(:));
       if mv == 0;
           break;
       end
       [a,b] = ind2sub(size(vs), idx);
       vs(a,b) = 0;
       if a-dh>=1 && a+dh<=size(wss,1) && b-dw>=1 && b+dw<=size(wss,2)
           aa = wss(a-dh:a+dh, b-dw:b+dw);
           aa = aa(~isnan(aa));
           bb = hss(a-dh:a+dh, b-dw:b+dw);
           bb = bb(~isnan(bb));
           medianw = median(aa);
           medianh = median(bb);
           w2 = round(medianw/2);
           h2 = round(medianh/2);
           if b-w2 <1 || a-h2 <1 || b+w2 > size(vs,2) || a+h2 > size(vs,1)
               continue;
           end
           bb1 = [max(1,b-w2),max(1,a-h2),min(size(vs,2), b+w2), min(size(vs,1),a+h2), cls,0]; 

           bbx = [bbx; bb1]; 
       end
        
    end
    if draw
        figure(657); imagesc(vs); axis image
    end
    bbx = bbx';
end
 


function bb = increasesize(bb, p)

    w = bb(4) - bb(2);
    h = bb(3) - bb(1);
    
    wn = w * p;
    hn = h * p;
    bb(1) = bb(1)-(wn-w)/2;
    bb(2) = bb(2)-(hn-h)/2;
    bb(3) = bb(3)+(wn-w)/2;
    bb(4) = bb(4)+(hn-h)/2;
    
    
end








