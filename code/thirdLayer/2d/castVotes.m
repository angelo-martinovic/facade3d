function vs =castVotes(vs, vp, vk)
 pts = size(vk);
 
 offset = round(size(vk,1)/2)-1;
 vp = round(vp);
 
 for i=1:size(vp,1)
     %just dont vote on the border for now
     if (vp(i,1) -offset) <= 0 || (vp(i,1) +offset) > size(vs,2) ||  (vp(i,2) -offset) <= 0 || (vp(i,2) +offset) > size(vs,1)
         continue;
     end
     %slice vs
     vs(vp(i,2)-offset:vp(i,2)+offset,vp(i,1)-offset:vp(i,1)+offset ) =  vs(vp(i,2)-offset:vp(i,2)+offset,vp(i,1)-offset:vp(i,1)+offset ) + vk;
     
 end


end