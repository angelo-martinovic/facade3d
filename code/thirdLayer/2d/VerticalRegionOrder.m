function yPos = VerticalRegionOrder(outImg,~)

    height = size(outImg,1);
%     width = size(outImg,2);
    maxOverlap = -Inf;
    yPos = [1 NaN NaN NaN height];
    
    projMap = zeros(height,7);
    for i=1:7
        projMap(:,i) = sum(outImg==i,2);
    end
    projMap(:,2) =  projMap(:,2)+projMap(:,1)+projMap(:,3);
    projMap(:,7) =  projMap(:,7)+projMap(:,4);
    
    % Coarse estimation
    skip = 10;
    
    for x1=1:skip:round(height/4)
%         fprintf('%d\n',x1);
        
        for x2=1:skip:round(height/4)
            
            for x3=round(height/2):skip:height-x1-x2

                overlap = sum(projMap(1:x1,6)) + ...
                       sum(projMap(x1+1:x1+x2,5)) +....
                       sum(projMap(x1+x2+1:x1+x2+x3,2)) + ...
                       sum(projMap(x1+x2+x3+1:end,7));

                if overlap>maxOverlap
                    maxOverlap = overlap;
                    yPos = [1 x1 x1+x2 x1+x2+x3 height];
                end
            end
            
            
        end
        
    end
    
    % Fine tuning
    x1_s = yPos(2);
    x2_s = yPos(3)-yPos(2);
    x3_s = yPos(4)-yPos(3);
    
    skip = skip/2;
    for x1=x1_s-skip:x1_s+skip
%         fprintf('%d\n',x1);
        if x1<1 || x1>height, continue; end
        for x2=x2_s-skip:x2_s+skip
            if x2<1 || x2>height, continue; end
            for x3=x3_s-skip:x3_s+skip
                if x3<1 || x3>height, continue; end
                overlap = sum(projMap(1:x1,6)) + ...
                       sum(projMap(x1+1:x1+x2,5)) +....
                       sum(projMap(x1+x2+1:x1+x2+x3,2)) + ...
                       sum(projMap(x1+x2+x3+1:end,7));

                if overlap>maxOverlap
                    maxOverlap = overlap;
                    yPos = [1 x1 x1+x2 x1+x2+x3 height];
                end
            end
            
            
        end
        
    end
      

end