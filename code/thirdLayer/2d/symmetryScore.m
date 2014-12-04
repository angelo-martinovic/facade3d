function [score,n] = symmetryScore( pos, x, center,im,visualize)
    
    minMatchDistance = 4;
    
    % Determine features left and right of the line
    if x<= center
        posright = pos(:,1) > x & pos(:,1) < 2*x;
        posleft =  pos(:,1) < x ;
    else
        posright = pos(:,1) > x;
        posleft =  pos(:,1) < x & pos(:,1) > x - (2*center-x);
    end
    n1 = sum(sum(posleft));
    n2 = sum(sum(posright));
    n = n1+n2;
    
    posright = pos(posright,:);
    posleft = pos(posleft,:);
    
    % Mirror the feature points on the right
    posrightm = posright;
    posrightm(:,1) = 2*x - posrightm(:,1);
    
    % Find distances between FPs on the left and the mirrored FPs from the
    % right side of the symmetry line
    distMat = pdist2(posleft,posrightm);
    
%     score = sum(sum( distMat < minMatchDistance ));
%     score = score / n;

    hits = (any(distMat<minMatchDistance));
    
    score = sum(hits)/size(hits,2);

    if visualize
        figure(123);imagesc(im); axis image;hold on;
        
        line([round(x) round(x)],[1 size(im,1)]);
        
        [ii,jj] = find(distMat<minMatchDistance);
        plot(posleft(ii,1),posleft(ii,2),'rx');
        plot(posright(jj,1),posright(jj,2),'bx');
        plot(posrightm(jj,1),posrightm(jj,2),'go');
        title(num2str(score));
    end
    
    
    
end
 









