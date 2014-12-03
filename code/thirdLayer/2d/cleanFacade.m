function [ out_bb ] = cleanFacade(bb,balcClass)
%delete windows under balconies
    rem = zeros(1, size(bb,2));
    balc = bb(:, bb(5,:) == balcClass);
    for i=1:size(balc,2)
       this_balc = balc(:,i);
       %this_area = bb_area(this_balc);
       for k=1:size(bb,2)
           if (sum(this_balc == bb(:,k)) ~= 6)
              ol = bb_overlap(this_balc, bb(:,k));
              if ol == bb_area(bb(:,k))
                 rem(k) = 1;     
              end
           end
       end
       
       
    end
    rem = ~ rem;
    out_bb = bb(:,rem);

end

function area = bb_overlap(a,b)
    area = 0;
    if (min(b(3), a(3))  > max(b(1),a(1))) && (min(b(4), a(4)) > max(b(2), a(2)))
        max_x1 = max(a(1), b(1));
        min_x2 = min(a(3), b(3));
        max_y1 = max(a(2), b(2));
        min_y2 = min(a(4), b(4));
        area = (min_x2 - max_x1) * (min_y2 -max_y1);
    end
   

end

function area = bb_area(a)
    area = (a(3) - a(1)) * (a(4)-a(2));
    
end