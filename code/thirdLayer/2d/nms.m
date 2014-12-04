function [p,q] = nms(boxes, pool, value_equal_class, value_different_class, ol_matrix, min_area_matrix)
    rem = zeros(1, size(pool,2));
    for i=1:size(boxes,2)
           a = boxes(:,i);
           label = boxes(5,i);
       for j=1:size(pool,2)
           if (rem(j) > 0)
               continue;
           end
           b = pool(:,j);
           %ol2 = bb_overlap(a,b);
           ol = ol_matrix(a(7), b(7));
           if ol ==0
               continue;
           end
           %minarea2 = min(bb_area(a) ,bb_area(b));
           minarea = min_area_matrix(a(7), b(7));
           %assert(minarea2 == minarea);
            if (label ~= pool(5,j))
                value = value_different_class;
            else
                value = value_equal_class;
                if (label == 3)
                    value = 0.7;
                end
            end
           if ol/minarea> value
           %if (min(x22, x21)  > max(x12,x11)) && (min(y22, y21) > max(y12, y11))
              rem(j)=rem(j)+1;
          
           end
           
           
       end
    end
    p = pool(:,~(rem>0));
    q = pool(:,(rem>0));
   

end


