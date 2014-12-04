
function output = elementSampling(origImg,sgmp,outImg,hyperParameters)   
    %% Initial elements
    disp('--Extracting initial elements...');
    
    % Probability map 
    PM_win = sgmp(:,:,hyperParameters.winClass);  
    PM_win = imfilter(PM_win, fspecial('gaussian',5,5)); % necessary?
    hyperParameters.PM_win = PM_win;
    
    % Get bounding boxes using minimum bounding rectangles
    [win_boundingBoxes, win_ddw, win_ddh] = getBoundingBoxesPool(hyperParameters.winClass,outImg,PM_win,hyperParameters);
    if isempty(win_boundingBoxes)
        warning('No windows found! Returning.');
        output = outImg;
        return;
    end
    if win_ddw>hyperParameters.parMaxMedWinSize
        warning('Sum ting wong. Windows too large. Returning.');
        output = outImg;
        return;
    end
    objMask = outImg==hyperParameters.winClass;
    
    % Element sizes
    hyperParameters.win_ddw = win_ddw;
    hyperParameters.win_ddh = win_ddh;
       
    initElements = win_boundingBoxes;

    if isfield(hyperParameters,'balcClass')
        PM_balc = sgmp(:,:,hyperParameters.balcClass); 
        PM_balc = imfilter(PM_balc, fspecial('gaussian',5,5));
        hyperParameters.PM_balc = PM_balc;
        
        [balc_boundingBoxes, balc_ddw,balc_ddh] = getBoundingBoxesPool(hyperParameters.balcClass, outImg, PM_balc, hyperParameters);
        
        hyperParameters.balc_ddw = balc_ddw;
        hyperParameters.balc_ddh = balc_ddh;
    
        if hyperParameters.legacy
            % Snap balconies to windows
            balc_boundingBoxes = checkBB(balc_boundingBoxes, win_boundingBoxes, win_ddw);
        end
        
        initElements = [initElements balc_boundingBoxes];
        
        objMask = objMask | outImg==hyperParameters.balcClass;
    end

    
    %% Vertical region order
    if hyperParameters.principles.verticalRegionOrder
        disp('--Principle: Vertical region order...');
        ypos = VerticalRegionOrder(outImg,sgmp);
    end

    %% Door
    if isfield(hyperParameters,'doorClass')
        PM_door = sgmp(:,:,hyperParameters.doorClass);
        [door_boundingBoxes, ~,~] = getBoundingBoxesPool(hyperParameters.doorClass, outImg, PM_door, hyperParameters);
        objMask = objMask | outImg==hyperParameters.doorClass;
    end
    
    if hyperParameters.principles.door
        disp('--Principle: Door...');
        door_boundingBoxes =  findDoor(ypos(4), PM_door, door_boundingBoxes, win_ddw*2, balc_ddh*2);
    end
    initElements = [initElements door_boundingBoxes];

    % Create background image (no-objects)
    backgroundImage = outImg;
    
    [ii,jj] = find(objMask==1);
    emptyPixels = [ii jj];
    [ii,jj] = find(objMask~=1);
    nonEmptyPixels = [ii jj];
    nonEmptyLabels = outImg(objMask~=1);
    
    idx = knnsearch(nonEmptyPixels,emptyPixels);
    emptyLabels = nonEmptyLabels(idx);
    backgroundImage(objMask==1) = emptyLabels;
    
%     figure;imagesc(backgroundImage);

    %% Symmetry
    assert(max(initElements(3,:))<=size(outImg,2),'Invalid object before symmetry');    
    if hyperParameters.principles.symmetry
        disp('--Principle: Symmetry...');
        if hyperParameters.principles.verticalRegionOrder
            % Restrict the search to a part of image
            [~, sympos]= symmetryDetect2(outImg,[ypos(2) ypos(4)]);%hyperParameters.visualize);
        else
            % Search on the entire image
            [~, sympos]= symmetryDetect(outImg,[1 size(outImg,1)]);%hyperParameters.visualize);
        end

        % Mirror the windows
        mbbx1 = getMirroredBBx(win_boundingBoxes, sympos, size(origImg,2));

        % Calculate their data support
        mbbx1(6,:)=sum(sum(PM_win(mbbx1(2,:):mbbx1(4,:), mbbx1(1,:):mbbx1(3,:)) )) ./ ((mbbx1(4,:)-mbbx1(2,:)+1) .* (mbbx1(3,:)-mbbx1(1,:)+1));
        
        % Add them to the pool
        initElements = [initElements mbbx1];

        if isfield(hyperParameters,'balcClass')
            mbbx2 = getMirroredBBx(balc_boundingBoxes, sympos, size(origImg,2));
            mbbx2(6,:)=sum(sum(PM_balc(mbbx2(2,:):mbbx2(4,:), mbbx2(1,:):mbbx2(3,:)) )) ./ ((mbbx2(4,:)-mbbx2(2,:)+1) .* (mbbx2(3,:)-mbbx2(1,:)+1));
            initElements = [initElements mbbx2];
        end

    end
    assert(max(initElements(3,:))<=size(outImg,2),'Invalid object after symmetry');   
   
   %% Sampling
   current_boxes = initElements;
   current_boxes = RemoveDuplicates(initElements,hyperParameters);
   current_boxes(1:4,:) = round( current_boxes(1:4,:) );
         
   %% Propose new elements
   if hyperParameters.principles.similarity
       disp('---Principle: Similarity...');
       current_windows = current_boxes(:,current_boxes(5,:) == hyperParameters.winClass);
       newBoxes = similarityVoting(origImg, current_windows',hyperParameters.visualize, win_ddw, win_ddh, hyperParameters.winClass, sum(current_boxes(5,:) == hyperParameters.winClass)+5);
       
       % Remove elements above the roof line
       if hyperParameters.principles.verticalRegionOrder && ~isempty(newBoxes)
           newCenters = (newBoxes(2,:)+newBoxes(4,:))/2;
           newBoxes(:,newCenters<ypos(2)) = [];
           newBoxes(:,newBoxes(4,:)>ypos(4)) = [];
           current_boxes = [current_boxes newBoxes]; 
       end

       if isfield(hyperParameters,'balcClass')
           current_balcs = current_boxes(:,current_boxes(5,:) == hyperParameters.balcClass);
           newBalcBoxes = similarityVoting(origImg, current_balcs', hyperParameters.visualize, balc_ddw, balc_ddh, hyperParameters.balcClass, sum(current_boxes(5,:) == hyperParameters.balcClass)+5);
           % Remove elements above the roof line
           if hyperParameters.principles.verticalRegionOrder && ~isempty(newBalcBoxes)
               newCenters = (newBalcBoxes(2,:)+newBalcBoxes(4,:))/2;
               newBalcBoxes(:,newCenters<ypos(2)) = [];
               newBalcBoxes(:,newBalcBoxes(4,:)>ypos(4)) = [];
               current_boxes = [current_boxes newBoxes];
           end
           current_boxes = [current_boxes newBalcBoxes]; 
       end

   end

   %% Find the best configuration 
   disp('---Optimizing...');
   [~, best_boxes] = optimizeConfiguration(current_boxes, outImg, hyperParameters);
   if isempty(best_boxes)
        output = outImg;
        return;
   end
   
   assert(max(best_boxes(3,:))<=size(outImg,2),'Invalid object after optimization');
 
   %% Alignment
   if hyperParameters.visualize
    figure(100);imagesc(LabelingFromBoxes(best_boxes,true,size(outImg),hyperParameters));axis equal;
   end
   if hyperParameters.principles.alignment
       disp('--Principle: Alignment...');
       opts = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','off');
       
       windows = best_boxes(:,(best_boxes(5,:) == hyperParameters.winClass));
       rest1 = best_boxes(:,(best_boxes(5,:) ~= hyperParameters.winClass));
       windows = fminunc(@(x) objfun3(x,PM_win,win_ddw, win_ddh), windows, opts);
       best_boxes = [windows rest1];
       
       if isfield(hyperParameters,'balcClass')
           balcs = best_boxes(:,(best_boxes(5,:) == hyperParameters.balcClass));
           rest2 = best_boxes(:,(best_boxes(5,:) ~= hyperParameters.balcClass));
           balcs = fminunc(@(x) objfun3(x,PM_balc,balc_ddw, balc_ddh), balcs, opts);
           best_boxes = [balcs rest2];
       end
   end
   if hyperParameters.visualize
    figure(101);imagesc(LabelingFromBoxes(best_boxes,true,size(outImg),hyperParameters));axis equal;
   end
   
   % Remove boxes violating the borders
   best_boxes(:,best_boxes(3,:)>size(outImg,2)) = [];
   
   assert(max(best_boxes(3,:))<=size(outImg,2),'Invalid object after alignment');
    
   %% Output
   disp('--Creating the output label map...');
   output = zeros(size(origImg,1), size(origImg,2));

   % Background
   if hyperParameters.principles.verticalRegionOrder
       output(ypos(1):ypos(2),1:size(origImg,2)) = 6;   % sky
       output(ypos(2)+1:ypos(3),1:size(origImg,2)) = 5; % roof
       output(ypos(3)+1:ypos(4),1:size(origImg,2)) = 2; % wall
       output(ypos(4)+1:ypos(5),1:size(origImg,2)) = 7; % shop;
%        output = backgroundImage;
   else
       output = backgroundImage;
%        output = outImg;
%        nonwinClasses = setdiff(1:7,hyperParameters.winClass);
%        [~,nonwins] = max(sgmp(:,:,nonwinClasses),[],3);
%        output(outImg==hyperParameters.winClass) = nonwins(outImg==hyperParameters.winClass);
   end
   
   % Objects
   wins = round(best_boxes(:,best_boxes(5,:) == hyperParameters.winClass));
   for i=1:size(wins,2)
       w = wins(:,i);
       if (w(4)-w(2)+1)*(w(3)-w(1)+1)<=hyperParameters.parMinObjectSize || w(4)-w(2)+1<=hyperParameters.parMinHeight || w(3)-w(1)+1<=hyperParameters.parMinWidth
         continue;
       end
       % Decrease the window size
%        w = w+[1; 0; -1; -0; 0; 0];
       output(w(2):w(4), w(1):w(3)) = hyperParameters.winClass;
   end
   
   if isfield(hyperParameters,'balcClass')
       balcs = round(best_boxes(:,best_boxes(5,:) == hyperParameters.balcClass));
       for i=1:size(balcs,2)
           w = balcs(:,i);
           if (w(4)-w(2)+1)*(w(3)-w(1)+1)<=hyperParameters.parMinObjectSize || w(4)-w(2)+1<=hyperParameters.parMinHeight || w(3)-w(1)+1<=hyperParameters.parMinWidth
             continue;
           end

           % Decrease the balcony size
%            w = w+[1; 0; -1; -0; 0; 0];
           output(w(2):w(4), w(1):w(3)) = hyperParameters.balcClass;
       end
   end
   
   if isfield(hyperParameters,'doorClass')
       doors = round(best_boxes(:,best_boxes(5,:) == hyperParameters.doorClass));
       % Enforce a door if the principle is active
       if isempty(doors) && hyperParameters.principles.door
           doors = door_boundingBoxes;
       end
       for i=1:size(doors,2)
           w = doors(:,i);
           if (w(4)-w(2)+1)*(w(3)-w(1)+1)<=hyperParameters.parMinObjectSize || w(4)-w(2)+1<=hyperParameters.parMinHeight || w(3)-w(1)+1<=hyperParameters.parMinWidth
             continue;
           end
           output(w(2):w(4), w(1):w(3)) = hyperParameters.doorClass;
       end
   end
   
   % Overlaid classes
   for i=1:length(hyperParameters.overrideClasses)
       output(outImg==hyperParameters.overrideClasses(i))=hyperParameters.overrideClasses(i);
   end
  
end

%% Modifies the balcony bounding boxes to correspond to windows
function ret = checkBB(balc, win, dw)
    % For every balcony
    for i=1:size(balc,2)
       b = balc(:,i);
       
       % Check balcony size
       if (b(3) - b(1)) < dw*2
           
           % Find the windows that might be close to the balcony
           w = win(:,win(3,:) > b(1) & win(1,:) < b(3) & win(4,:) < b(4));
           dmin = Inf;
           
           % Find the window closest to the balcony
           for kk=1: size(w,2)
              d = abs(w(4,kk) -b(4));
              if d< dmin
                 dmin = d;
                 this_win = w(:,kk);
              end
           end
           % If a window is found, resize the balcony size to fit the window
           if ~isempty(w)
            balc(1,i) = this_win(1);
            balc(3,i) = this_win(3);
           end
       end
    end
    ret = balc;

end

function [boundingBoxes, medianw, medianh, poolBB] = getBoundingBoxesPool(classidx, outImg, pm, hyperParameters)
    if hyperParameters.legacy
        [boundingBoxes, medianw, medianh, poolBB] = getBoundingBoxesPoolOld(classidx, outImg, pm);
    else
        [boundingBoxes, medianw, medianh, poolBB] = getBoundingBoxesPoolNew(classidx, outImg, pm, hyperParameters);
    end
end

% Extracts bounding boxes with a pool of alternatives
function [boundingBoxes, medianw, medianh, poolBB] = getBoundingBoxesPoolOld(classidx, outImg, pm)
    mask = (outImg ==classidx);
    show = 0;
    bw =  imdilate(mask, [1 1 1; 1 0 1; 1 1 1]);
    bw =  imdilate(bw, [1 1 1; 1 0 1; 1 1 1])- mask;
    if show==1
        figure(123); axis image;
        imagesc(bw);
    end
    [conComp, n] = bwlabel(mask);
    widths = [];
    heights = [];
    boundingBoxes = zeros(6,n); np=0;
    poolBB = zeros(6,3*n);npp = 0;
    for i=1:n

       t = (conComp ==i);
       [r,c] = find(t);
      
       x1 = min(c);
       y1 = min(r);
       x2 = max(c);
       y2 = max(r);
       widths(i) = x2-x1;
       heights(i) = y2-y1;
       
       if widths(i)<=1 || heights(i)<=1
           continue;
       end

       nf = sum(sum(pm(y1:y2, x1:x2)));
       
       %search for max until mass < 80% of nf
       maxy = 1;
       maxl = -inf;
       linev = zeros(1,y2-1-y1); c=1;
       for yy = y1:y2-1
           m = sum(sum(pm(yy:y2, x1:x2)))/nf;
           if m<0.6
               break;
           end
           if yy < 3 || yy+2 > size(bw,1)
               line = sum(sum(bw(yy:yy,x1:x2)))/((x2-x1));
           else
                line = sum(sum(bw(max([1 yy-2]):min([yy+2 size(bw,1)]),x1:x2)))/((x2-x1)*5);
           end
           linev(c) = line;
           if line>maxl
               maxl = line;
               maxy = yy;
           end
           c = c+1;
       end
       mlinev = max(linev);
       m1t = find(linev ==mlinev,1);
       linev(max(1,m1t-2):min(m1t+2,size(linev,2))) = 0;
       m2t = find(linev ==max(linev),1);
       if mlinev*0.75 > linev(m2t)
          m2t = 0; 
       else
           m2t = m2t+y1;
       end
       
       y1 = maxy;
       maxy = 1;
       maxl = -inf;
       linev = zeros(1,y2-1-y1); c=1;
       for yy = y2:-1:(y1-1)

           m = sum(sum(pm(y1:yy, x1:x2)))/nf;
           if m<0.6
               break;
           end
           line = sum(sum(bw(max([1 yy-2]):min([yy+2 size(bw,1)]),x1:x2)))/((x2-x1)*5);
           linev(c) = line;
           if line>maxl
               maxl = line;
               maxy = yy;
           end
          c = c+1; 
       end
       y2=maxy;
       mlinev = max(linev);
       m1b = find(linev ==mlinev,1);
       
       linev(max(1,m1b-2):min(m1b+2,size(linev,2))) = 0;
       m2b = find(linev ==max(linev),1);
       if mlinev*0.75 > linev(m2b)
          m2b = 0;
       else
           m2b = y2 -m2b;
       end

       np = np + 1;
       score = sum(sum(pm(y1:y2, x1:x2) )) / ((y2-y1) * (x2-x1));
       boundingBoxes(:, np) = [ x1,y1, x2,y2, classidx, score];
       
       if m2t ~= 0
          npp = npp + 1;
          score = sum(sum(pm(m2t:y2, x1:x2) )) / ((y2-m2t) * (x2-x1));
          poolBB(:, npp) = [ x1,m2t, x2,y2, classidx, score];
       end
       if m2b ~=0
          npp = npp + 1;
          score = sum(sum(pm(y1:m2b, x1:x2) )) / ((m2b-y1) * (x2-x1));
          poolBB(:, npp) = [ x1,y1, x2,m2b, classidx, score];
       end
        if m2b ~=0 && m2t ~= 0
          npp = npp + 1;
          score = sum(sum(pm(m2t:m2b, x1:x2) )) / ((m2b-m2t) * (x2-x1));
          poolBB(:, npp) = [ x1,m2t, x2,m2b, classidx, score];
       end
       
    end
    poolBB = poolBB(:,1:npp);
    medianw =round(median(widths)/2.0);
    medianh = round(median(heights)/2.0);
    boundingBoxes = boundingBoxes(:, 1:np);
end

%% Extracts bounding boxes with a pool of alternatives
function [boundingBoxes, medianw, medianh] = getBoundingBoxesPoolNew(classidx, outImg, pm, hyperParameters)
    % Mask out only the target class
    mask = (outImg ==classidx);
    
    boundingBoxes = [];
    
    for strelSize = [1 20 80] % [1 10 20 40]

            % Closing operation
            if strelSize>=80
                maskClosed = imdilate(imerode(mask,strel('line',strelSize,0)),strel('line',strelSize,0));
            else
                maskClosed = imdilate(imerode(mask,strel('square',strelSize)),strel('square',strelSize));
            end
            
            parMinObjectSize = hyperParameters.parMinObjectSize;  % in pixels

            % Get connected components
            [conComp, n] = bwlabel(maskClosed);
            
            % For each component
            for i=1:n

               % Get component pixels
               t = (conComp ==i);
               [r,c] = find(t);

               % Get component extent
               x1 = min(c);
               y1 = min(r);
               x2 = max(c);
               y2 = max(r);
               width = x2-x1+1;
               height = y2-y1+1;

               % Ignore small elements
               if width<=hyperParameters.parMinWidth || height<=hyperParameters.parMinHeight
                   continue;
               end

               if width*height<=parMinObjectSize
                   continue;
               end

               % Add element
               score = sum(sum(pm(y1:y2, x1:x2) )) / ((y2-y1) * (x2-x1));
               boundingBoxes=[boundingBoxes [x1;y1; x2;y2; classidx; score]];
            end
            
    end
    
    boundingBoxes = unique(boundingBoxes','rows')';
    if ~isempty(boundingBoxes)
        widths = boundingBoxes(3,:)-boundingBoxes(1,:)+1;
        heights = boundingBoxes(4,:)-boundingBoxes(2,:)+1;

        delIdxs = widths<=hyperParameters.parMinWidth| heights<=hyperParameters.parMinHeight | widths.*heights<=parMinObjectSize;
        boundingBoxes(:,delIdxs) = [];
        widths(:,delIdxs) = [];
        heights(:,delIdxs) = [];

        medianw = round(median(widths)/2);%/2.0);    % half the median of widths
        medianh = round(median(heights)/2);%/2.0);
    else
        medianw = NaN;
        medianh = NaN;
    end

end

function current_boxes = RemoveDuplicates(initElements,hyperParams)
%     figure(100);imagesc(LabelingFromBoxes(initElements,false,size(outImg),hyperParams));axis equal;
    removeFlags = zeros(size(initElements,2),1);
    elemTypes = unique(initElements(5,:));
    for i=elemTypes
        indices = find(initElements(5,:)==i);
        elems = initElements(:,indices)';
        
        dmats = cell(4,1);
        for j=1:4
%             elems = elems(:,1:4);

            % Find distances between pairs
            D = pdist(elems(:,j),@(x,y)BoxDistance(x,y,i,j));
            dmats{j} = squareform(D)==0;
        end
        dmat = ~(dmats{1} & dmats{2} & dmats{3} & dmats{4});
        
        % Find the ones that are zero
        inds = find(dmat==0 & triu(ones(size(dmat)),1)==1);
        
        % Remove the duplicate elements
        [~,jj]=ind2sub(size(dmat),inds);
        removeFlags(indices(jj)) = 1;
    end
    
    current_boxes = initElements(:,~removeFlags);
%     figure(101);imagesc(LabelingFromBoxes(current_boxes,false,size(outImg),hyperParams));axis equal;
    
    function d = BoxDistance(XI,XJ,objectType,direction)
        % XI is a row
        % XJ can be multiple rows
        absdist = abs(bsxfun(@minus,XI,XJ));
        if objectType==1
            ddw = hyperParams.win_ddw;
            ddh = hyperParams.win_ddh;
        else
            ddw = hyperParams.balc_ddw;
            ddh = hyperParams.balc_ddh;
        end
        
        if direction==1 || direction==3
            absdist(absdist<ddw)=0;
        else
            absdist(absdist<ddh)=0;
        end
        
        d = absdist;
%         d = sum(absdist,2);
    end

end




