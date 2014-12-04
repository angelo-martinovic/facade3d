% y - shopLine
% pm - noisy labeling
% bb - current door boxes
% winw - median window width
% winh - median window height
function [door] =findDoor(y,pm,bb,winw, ~)
cc =0;
% mass =1;
door = zeros(6,1);

% If there are already doors
if ~isempty(bb)
    
    %Select only the ones under the shop line
    bb = bb(:, bb(2,:)>y);
    
    % Check their widths and heights
    for i=1:size(bb,2)
        h = bb(4,i) -bb(2,i);
        w = bb(3,i) -bb(1,i);
        
        % If height>width and width at least 0.66 window width
        % Leave that door
        if h/w > 1 && w>winw/1.5
            cc = cc+1;
            door(:,cc) = bb(:,i);
        end
        
    end
    % If at least one good door existed, we're done
    if cc>0
        return;
    end
end

% No doors
height = size(pm,1);
width = size(pm,2);

% Search space
% 1.1 * window width < door width  < 3.5 * window width
w= round(winw*1.1): round(winw*3.5);

w(w>width/2) = [];

while(length(w)>50)
    w = w(1:2:end);
end

% 1.5 < height/width ratio < 2.5
h= 1.5:0.05:2.5 ;
% mass =0;
maxmass = -inf;

% For every width
for wi =1:size(w,2)
    ww = w(wi);
    
    startPos = 1:width-ww;
    while(length(startPos)>200)
        startPos = startPos(1:2:end);
    end
    % For every height
    for hi = 1:size(h,2)
        hh = round(h(hi)*w(wi));
        
        if hh>=height
            continue;
        end
        % For every horizontal position
        for i=startPos
            
            % Sum the probability mass in the rectangle and normalize
            mass = sum(sum(pm(height-hh:height, i:i+ww)))/(ww*hh); 
            
            % If the door is the best so far, overwrite the previous one.
            if mass> maxmass
                maxmass = mass;
                door(:,1)= [i (height-hh) (i+ww) height, 4, mass]';
            end
            
        end
    end
end




end



