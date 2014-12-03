function [maxscore, maxpos]= symmetryDetect2(im, symmetryRange)
        
%     sift =0;
%     harris =1;
%    
% 
%     I = double(rgb2gray(im));
% 
%     
%     if harris ==0
%       if (sift ==0)
%              [featureVectors,featureCoords] = getSSIMFeatNPos(I);
% 
%       else
%              [featureVectors,featureCoords] = getSIFTFeatNPos(I);
%       end
%     else
% 
% %          sd = 0.7;
%          si = 2.0;
% sd = 0.001;
% % si = 1.0;
%          idx = vl_localmax( vl_harris( vl_imsmooth( I, sd ), si ) ) ;
%          [i,j] = ind2sub( size(I), idx );
% 
%          C = [j;i];
%         
%         featureCoords = C';
%     end
%     if symmetryRange(1) ~=inf && symmetryRange(2) ~= inf
%         
%         featureCoords = featureCoords(featureCoords(:,2)>symmetryRange(1),:);
%         featureCoords = featureCoords(featureCoords(:,2)<symmetryRange(2),:);
%     end
    im = im(symmetryRange(1):symmetryRange(2),:);

    maxscore = -inf;
    maxpos = -1;
    from = size(im,2)/2;
    from = from - from/4;
    to = size(im,2)/2;
    to = to + to/4;
    from = round(from);
    to = round(to);
    scores = zeros(to-from+1,1);
    i=1;
    for x=from:to

       score = symScore2(im,x);%symmetryScore(featureCoords, x, size(im,2)/2,im,visualize);
       scores(i) = score;
       i=i+1;
       if score> maxscore
          maxscore = score;
          maxpos = x;
       end
        
    end
%     figure;plot(scores);

end

function [score] = symScore2(im,x)
    W = size(im,2);
    
    % Even width
    if mod(W,2)==0
        if x<=W/2
            %Left
            leftPart = im(:,1:x);
            rightPart = im(:,x+1:2*x);
        else
            %Right
            rightPart = im(:,x:size(im,2));
            leftPart = im(:,x-(size(im,2)-x)-1:x-1);
        end
        
    % Odd width
    else
        if x<W/2
            %Left
            leftPart = im(:,1:x);
            rightPart = im(:,x+1:2*x);
        elseif x>ceil(W/2)
            %Right
            rightPart = im(:,x:size(im,2));
            leftPart = im(:,x-(size(im,2)-x)-1:x-1);
        else
            % Exactly in the center
            leftPart = im(:,1:x-1);
            rightPart = im(:,x+1:end);
        end
        
        
    end
    leftPartMirrored = leftPart(:,end:-1:1);
    score = sum(leftPartMirrored(:)==rightPart(:))/numel(rightPart);

end