function bb_out = nms1(bb, value_equal_class, value_different_class, ol_matrix, min_area_matrix)

    % Elements to remove
    rem = zeros(1, size(bb,2));
    
    % Check pairs of bounding boxes
    for i=1:size(bb,2)
       
       % Element 1
       a = bb(:,i); 
       for j=i+1:size(bb,2)
            
           % Element 2
           b = bb(:,j);
           ol = ol_matrix(a(7), b(7));
           
           % Check if the elements overlap
           if ol ==0
               continue;
           end
           
           % Size of the smaller element
           minarea = min_area_matrix(a(7), b(7));
           
           % Check classes of elements
           if a(5) ~=b(5)
               value = value_different_class;
           else
               value = value_equal_class;
               % Hack for balconies, for some reason
               if (a(5) == 3)
                    value = 0.7;
                end
           end
               
           % Intersection over minimum area > threshold?
           if ol/minarea> value
               
              % If yes, remove element j
              rem(j)=rem(j)+1;
           end
           
           
       end
    end
    bb_out = bb(:,~(rem>0));
    
end