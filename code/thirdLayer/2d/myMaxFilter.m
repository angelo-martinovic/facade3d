function B=myMaxFilter(im, window_w, window_h)




    %im = im2;%imfilter(im2, fspecial('gaussian',15,15));
    window_w = round(window_w);
    window_h = round(window_h);
    ww = round(window_w);
    wh = round(window_h);
    [m,n]=size(im); B=zeros(size(im));
    ma = 0;
    im(1:wh+1,:) = 0;
    im(:, 1:ww+1) = 0;
    im(:,max([1 end-ww-1]):end) = 0;
    im(max([1 end-ww-1]):end,:) = 0;
    %for ii=window_h+1:m+1-window_h
    while true 
        [mv,idx] = max(im(:));
        [a,b] = ind2sub(size(im), idx);
        if mv == 0
            break;
        end
        B(a,b) = mv;
        im(a-wh:a+wh, b-ww:b+ ww) = 0;
    end
        
%     for ii=1:m
%         %for kk=window_w+1:n+1-window_w
%         for kk=1:n
%             if ii<window_h+1 || ii > m+1-window_h || kk < window_w+1 || kk> n+1-window_w
%                 B(ii,kk) = 0;
%                 continue
%             end
%             ma = 0;
%             %ind1 = sub2ind(size(im), ii-window_h:ii+window_h-1);
%             %ind2 = sub2ind(size(im), kk-window_w:kk+window_w-1);
%             crop = im(ii-window_h:ii+window_h-1,kk-window_w:kk+window_w-1);
%             ma = max(max(crop));
%             if (im(ii,kk) ~= ma) || ma ==0
%                 B(ii,kk) = 0;
%             else
%                 B(ii,kk) = 1;
%             end
%         end
%     end
%     %im = im2;%imfilter(im2, fspecial('gaussian',15,15));
%     window_w = round(window_w);
%     window_h = round(window_h);
%     [m,n]=size(im); B=im;
%     ma = 0;
%     %for ii=window_h+1:m+1-window_h
%     for ii=1:m
%         %for kk=window_w+1:n+1-window_w
%         for kk=1:n
%             if ii<window_h+1 || ii > m+1-window_h || kk < window_w+1 || kk> n+1-window_w
%                 B(ii,kk) = 0;
%                 continue
%             end
%             ma = 0;
%             %ind1 = sub2ind(size(im), ii-window_h:ii+window_h-1);
%             %ind2 = sub2ind(size(im), kk-window_w:kk+window_w-1);
%             crop = im(ii-window_h:ii+window_h-1,kk-window_w:kk+window_w-1);
%             ma = max(max(crop));
%             if (im(ii,kk) ~= ma) || ma ==0
%                 B(ii,kk) = 0;
%             else
%                 B(ii,kk) = 1;
%             end
%         end
%     end



end

            
