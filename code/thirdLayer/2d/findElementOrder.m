function ypos = findElementOrder(outImg, pm, win_ddh)
    sky = 6;
    roof=5;
    facade = 2;
    shop =7;
    chimney=8;
    window =1;
    balcony = 3;
    door=4;
    %
    delmask =zeros(size(outImg));
    mask = (outImg ==chimney);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    
    delmask(:) = mask(:) + delmask(:);
    mask = (outImg ==window);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    
    delmask(:) = mask(:) + delmask(:);
        mask = (outImg ==balcony);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    
    delmask(:) = mask(:) + delmask(:);
        mask = (outImg ==door);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    mask =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    delmask(:) = mask(:) + delmask(:);
    delmask = delmask>0;
    

    % Boundaries of sky, roof, wall, shop
    bwn = zeros(size(pm,1), size(pm,2),4);
    
    mask = (outImg ==sky);
    bwsky =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    bwn(:,:,1) =  imdilate(bwsky, [1 1 1; 1 0 1; 1 1 1])- mask;

    mask = (outImg ==roof);
    bwroof =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    bwn(:,:,2) =  imdilate(bwroof, [1 1 1; 1 0 1; 1 1 1])- mask;
    
    mask = (outImg ==facade);
    bwfacade =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    bwn(:,:,3) =  imdilate(bwfacade, [1 1 1; 1 0 1; 1 1 1])- mask;
    
    mask = (outImg ==shop);
    bwshop =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    bwn(:,:,4) =  imdilate(bwshop, [1 1 1; 1 0 1; 1 1 1])- mask;
    
    %search best line by gradient and probablility mass
    pmn = zeros(size(pm,1), size(pm,2),4);
    pmn(:,:,1) = imfilter(pm(:,:,sky), fspecial('gaussian',5,5));
    pmn(:,:,2) = imfilter(pm(:,:,roof), fspecial('gaussian',5,5));
    pmn(:,:,3) = imfilter(pm(:,:,facade), fspecial('gaussian',5,5));
    pmn(:,:,4) = imfilter(pm(:,:,shop), fspecial('gaussian',5,5));   
    h = size(outImg,1);
    spmn = sum(pmn,2);
    
    % Remove boundaries towards object classes
    for i=1:4
       t = bwn(:,:,i);
       t(delmask) = 0;
       bwn(:,:,i) = t;
    end
   
%     maxe=-inf;
%     max1=-inf;
%     for i=5:floor(h/2)
%         
%         e = sum(spmn(1:i,1),1);
%         e= e/sum(sum(spmn(:,1)));
%         if e > maxe
%             maxe=e
%             max1 = i
%         end
%     end
%     
%     
%     
%     %initialization
   % ypos = [1*h/10 2*h/10 8*h/10];
   % yold = ypos;
    
  %  fminunc(@(ypos) bestSplitobj(ypos,outImg,pmn), ypos);
    
    
    mine=inf;
    min1=inf;
    for i=5:h/8
        ypos=[i h/5 h/1.19];
        e = bestSplitobj(ypos,outImg,pmn,bwn);
        if e < mine
            mine=e;
            min1 = i;
        end
    end
   
    min2=inf;
    mine=inf;
    for i=min1+1:min1+1.5*win_ddh%h/2
        ypos=[min1 i h/1.19];
        e = bestSplitobj(ypos,outImg,pmn,bwn);
        if e < mine
            mine=e;
            min2 = i;
        end
    end
    min3=inf;
    mine=inf;
    for i=min2+1:h-1
        ypos=[min1 min2 i];
        e = bestSplitobj(ypos,outImg, pmn,bwn);
        if e < mine
            mine=e;
            min3 = i;
        end
    end
    
 
    %figure; imagesc(outImg);
    %line([1 size(outImg,2)], [min1 min1] ,'LineWidth',4,'Color',[.11 .98 .98])
    %line([1 size(outImg,2)], [min2 min2],'LineWidth',4,'Color',[.11 .98 .98])
    %line([1 size(outImg,2)], [min3 min3],'LineWidth',4,'Color',[.11 .98 .98])
    ypos = [1 min1 min2 min3 h];
end


function e = bestSplitobj(ypos, outImg, pmn, bwn)
    e=0;
   
    %maximize size
    %maximize horizontal line matching
    %ypos = round(ypos);
    %maximize probability density
    spmn = sum(pmn,2);
    sbwn = sum(bwn,2);
    h = size(outImg,1);
    w = size(outImg,2);
    ypos = round([1 ypos]);
    a1 =  sum(spmn(1:ypos(2),1));
    a1 = a1/sum(sum(spmn(:,1)));
    b1 =  sum(spmn(ypos(2):ypos(3),2));
    b1 = b1/sum(sum(spmn(:,2)));
    c1 =  sum(spmn(ypos(3):ypos(4),3));
    c1 = c1/sum(sum(spmn(:,3)));
    d1 =  sum(spmn(ypos(4):h,4));
    d1 = d1/sum(sum(spmn(:,4)));
    e1 = (a1+b1+c1+d1);
    a2 =  sum(sbwn(ypos(2),1));
    a2 = a2/w;
    b2 =  sum(sbwn(ypos(3),2));
    b2 = b2/w;
    c2 =  sum(sbwn(ypos(4),3));
    c2 = c2/w;
    %e = a1*a2+b1*b2+c1*c2+d1
    e2 = a2+b2+c2;
    e = e1+e2;
    
   
    e = -e;
   
%     a =  sum(spmn(1:floor(ypos(2)),1));
%     a=  a + (ypos(2) - floor(ypos(2)))*spmn(floor(ypos(2))+1,1);
%     a = a/sum(sum(spmn(:,1)));
%     %a = a/((ypos(2)-1)*size(pmn,2));
%     
%     b =  sum(spmn(floor(ypos(2))+1:floor(ypos(3)),2));
%     b = b + (ypos(2) - floor(ypos(2)))*spmn(floor(ypos(2)),2);
%     b=  b + (ypos(3) - floor(ypos(3)))*spmn(floor(ypos(3))+1,2);
%     %b=b*h;
%     b = b/sum(sum(spmn(:,2)));
%     %b = b/((ypos(3)-ypos(2))*size(pmn,2));
%     
%     c =  sum(spmn(floor(ypos(3))+1:floor(ypos(4)),3));
%     c = c + (ypos(3) - floor(ypos(3)))*spmn(floor(ypos(3)),3);
%     c=  c + (ypos(4) - floor(ypos(4)))*spmn(floor(ypos(4))+1,3);
%     %c = c/((ypos(4)-ypos(3))*size(pmn,2));
%     c = c/sum(sum(spmn(:,3)));
%     
%     d =  sum(spmn(floor(ypos(4))+1:h,4));
%     d = d + (ypos(4) - floor(ypos(4)))*spmn(floor(ypos(4)),4);
%     %d=  d + (ypos(5) - floor(ypos(5)))*spmn(floor(ypos(5))+1,4);
%     %d = d/((h-ypos(4))*size(pmn,2));
%     d = d/sum(sum(spmn(:,4)));
%     e = a+b+c+d;
%     
%     %/(ypos(2)-ypos(1)*size(pmn,2)); 
%     %e = e+ sum(pmn(ypos(2):ypos(3),:,2))/(ypos(3)-ypos(2)*size(pmn,2));
%     %e = e+ sum(pmn(ypos(3):ypos(4),:,3))/(ypos(4)-ypos(3)*size(pmn,2));
%     %e = e+ sum(pmn(ypos(4):ypos(5),:,4))/(ypos(5)-ypos(4)*size(pmn,2));
%     e = -e;

end