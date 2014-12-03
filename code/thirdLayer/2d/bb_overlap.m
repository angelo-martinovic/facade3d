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