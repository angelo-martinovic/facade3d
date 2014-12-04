function e = objfun3(p, ~,ddw,ddh,~)
%     e1 = 0;
%     e2 = 0;
    %dd = 35;
%     w = 0;
%     for i = 1 : size(p,2)
%         %calc median w and h
%         w(i) = p(3,i)-p(1,i);
%         h(i) = p(4,i)-p(2,i);
%        
%     end
    %dd= median(w)*2;
    

    %normalizationFactor = size(p,2)*(size(p,2)-1)/2;
%     maxval = 0;
%     ascore = 0;
%     for i = 1 : size(p,2)
%         ascore = ascore + alignmentScore([p(:,1:i-1) p(:,i+1:end)],p(:,i),3,3);
%         for j = i + 1 : size(p,2)
            
%             whorTop = 1;
%             whorWidth = 1;
%             if i ==1 && j ==2
%                 maxval = maxval + tukey(ddw+1, ddw);    
%                 maxval = maxval + tukey(ddw+1, ddw); 
%                 maxval = maxval + whorTop*tukey(ddh+1, ddh);
%                 maxval = maxval + tukey(ddh+1, ddh);
%                 maxval = maxval + whorWidth* tukey(ddw*4+1, ddw*4);
%             end
            
            %vertical allignment
            %e1 = e1 + tukey(p(1,i)-p(1,j), ddw);
            %e1 = e1 + tukey(p(3,i)-p(3,j), ddw);
            
            %horizontal allignment
%             e1 = e1 + tukey(p(2,i)-p(2,j), ddh);
            
%             e1 = e1 + tukey(p(4,i)-p(4,j), ddh);
            
            
            %horizontal widths
%             d1 = max(p(2,i), p(2,j));
%             d2 = min(p(4,i), p(4,j));
%             w1 = p(3,i)-p(1,i);
%             w2 = p(3,j)-p(1,j);
%             e1 = e1 + whorWidth*weightWidths2(d1,d2,w1,w2,ddw*4);
            
            
           
%         end
        %maxpaircount = max(paircount, maxpaircount);
        
     %   e2 = e2 + getPD(round(p(1,i)), round(p(2,i)), round(p(3,i)-p(1,i)), round(p(4,i)-p(2,i)), PM,max(ddw,ddh));
        
%     end
%     ascore = ascore/size(p,2);
%     
%     e1 =e1/(normalizationFactor*maxval);
%     if (e1 >= 1)
%         disp('ERROR');
%         e1
%         error('e1 must be smaller/equal 1');
%     end
%             
%     if exist('useDT')
%     e = e1 + 0.05*e2;
%     else
%         e=e1;
%     end
%     fun = @(x) tukey(x,ddh);
%     e = sum(arrayfun(fun,pdist(p(2,:)')))+sum(arrayfun(fun,pdist(p(4,:)')));
%     e = sum( tukey2(  pdist(p(1,:)'),ddh  ) )+sum( tukey2(  pdist(p(3,:)'),ddh  ) );

    e = sum( tukey2(  pdist(p(1,:)'),ddw  ) )+sum( tukey2(  pdist(p(3,:)'),ddw  ) ) ...
      + sum( tukey2(  pdist(p(2,:)'),ddh  ) )+sum( tukey2(  pdist(p(4,:)'),ddh  ) ) ;
    
end
% function e = weightWidths2(d1, d2, w1,w2, dd)
%     if d1>d2
%         e = tukey(dd+1, dd); %tukey(4001,400);     
%     else
%        e = tukey(w1-w2, dd); % exp(0.1*abs(w1-w2));%:
%     end
% 
% 
% end
% 
% 
% function e = weightWidths(dist, w1,w2, dd)
%     if abs(dist) > dd
%         e = tukey(dd+1, dd); %tukey(4001,400);     
%     else
%        e = tukey(w1-w2, 40); % exp(0.1*abs(w1-w2));%:
%     end
% 
% 
% end

% function psi = tukey(e, k)
%     if abs(e) < k
%         psi =  k^2/6 * (1 - (1-(e/k)^2)^3);
%     else
%         psi = k^2/6;
%     end
% end

% e can be a vector
function psi = tukey2(e, k)
    maxval = k^2/6;
    
    psi = maxval * (1 - (1-(e./k).^2).^3);
    
    psi(abs(e)>=k) = maxval;
end

% function energy = getPD(x,y,w,h,PM,dd)
% 
%     if isnan(x) || isnan(y) || isnan(w) || isnan(y) || w*h==0 || x<1 || y<1
%         energy =tukey(dd+1, dd); 
%         return;
%     end
%     if (y+h > size(PM,1)) || x+w > size(PM,2)
%         energy = tukey(dd+1, dd); 
%     else
%         energy =1- sum(sum(PM(y:y+h,x:x+w)))/(w*h);
%     end
% end

