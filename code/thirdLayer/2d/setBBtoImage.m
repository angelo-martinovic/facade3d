function bb = setBBtoImage(bb,w,h)
    for i=1:size(bb,2)
       bb(1,i) = max(bb(1,i),1);
       bb(3,i) = min(bb(3,i),w);
       bb(2,i) = max(bb(2,i),1);
       bb(4,i) = min(bb(4,i),h);
        
    end

end