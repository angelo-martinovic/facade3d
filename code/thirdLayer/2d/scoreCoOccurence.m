function score =scoreCoOccurence(boxes, dh)

    score = 0;
    balcs = boxes(:, boxes(5,:) == 3);
    wins = boxes(:, boxes(5,:) == 1);
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
%     score = score + sum(hitmap>1); % VERSION M
    
    score = score/size(balcs,2) + sum(hitmap>1)/size(wins,2);

end

