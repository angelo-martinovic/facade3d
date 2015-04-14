function score =scoreCoOccurence(boxes, hyperParameters)

    score = 0;
    balcs = boxes(:, boxes(5,:) == hyperParameters.balcClass);
    wins = boxes(:, boxes(5,:) == hyperParameters.winClass);
    hitmap = zeros(1, size(wins,2));
    for i=1:size(balcs,2)
        b = balcs(:,i);
        h = b(4) - b(2);
        
        %above
        %mask for not overlapping vertically
        mask = ~(wins(3,:) < b(1) | wins(1,:) > b(3)  );
        %mask for touching
        mask2 =  (abs(b(2) - wins(4,:) ) < (h* 0.7)) | (abs(b(4) - wins(4,:) ) < (h* 0.7));
        w = wins(:,mask & mask2);
        %figure(999); imagesc(im); axis image;
        %drawAllRects(w,999);
        %drawAllRects(b,999);
        if isempty(w) ==1
           score = score +1; 
        else
            hitmap = hitmap + (mask & mask2);
        end
        
        
        
    end
    
    if hyperParameters.legacy
        score = score + sum(hitmap>1);
    else
        score = score/size(balcs,2) + sum(hitmap>1)/size(wins,2);
    end

end

