function spinImgs = compSpinImages(model, radius, imgW, minNeighbors)
% Computer spin images
%
% model - N x 3 matrix of points
% radius - radius of spin image
% imgW - number of bins
% minNeighbors - minimum number of neighbors before returning a valid (non-empty) spin image

imgH = imgW/2;
N = size(model,1);
%[cp,dist,root] = kdtree(model, []);

[~,~,root] = kdtree(model, []);

spinImgs = zeros(2*imgH,imgW,N);
% spinImgs = rand(2*imgH,imgW,N);
% return;
neighborRadius = radius * sqrt(2); % neighbor search radius has to be larger because the spin image is cylindrical
    
% loop over points
for i=1:N
   %[ i N]
%    fprintf('\b\b\b\b\b\b\b\b\b\b\b\b\b%5iK-%5iK',round(i/1e3),round(N/1e3));
   pt = model(i,:);
   %pt = model(:,i)';
   spinImg = spinImgs(:,:,i);
   
   % calculate neighbors of current point
   neighbors = kdrangequery(root,pt,neighborRadius - 1e-7);
   
   if size(neighbors,1) >= minNeighbors
      % first we compute the normal vector
      
      % Old version
       % compute PCA
%        coeff = princomp(neighbors);
         %coeff = pca(neighbors');
      
       % third component is surface normal
%       normal = coeff(:,3);
     
      % New version - much faster
      normal = normnd(neighbors);
      
      if dot(normal, pt) < 0
         normal = -normal;
      end

      % now we compute the spin image
      nn = length(neighbors);
      diffs = (neighbors - repmat(pt,nn,1))./radius;
      lens = sqrt(dot(diffs',diffs'))';

      % y-coord is dot prod between normal and vector to neighbor
      yvals = dot(repmat(normal',nn,1),diffs,2);
      
      % x-coord is distance of the neighbor to the normal line
      xvals = sqrt(lens.^2 - yvals.^2);

      % only add points if they are actually in the spin image
      xyvals = [xvals yvals];
      xyvals = xyvals(abs(xvals) < 1 & abs(yvals) < 1,:);
      nPts = length(xyvals);
      xvals = xyvals(:,1); yvals = xyvals(:,2);
      xInds = round(xvals.*(imgW-1))+1;
      yInds = imgH + round(yvals.*(imgH-1))+1;

      if(nnz(xInds < imgW & yInds < 2*imgH) == 0)
         disp('error computing spin image!');
      else
         
         % Old version
%          inds = sub2ind(size(spinImg),xInds,yInds);
%          for j=1:length(inds)
%             spinImg(inds(j)) = spinImg(inds(j)) + 1;
%          end
         
         % New version
         spinImg = spinImg + accumarray([xInds yInds],1,size(spinImg));

      end
      
      % if only 1 bin is occupied, then this corresponds to the query point
      % --> neighborhood is too sparse, i.e. no spin image
      if(nnz(spinImg >= 1) == 1)
         spinImg = zeros(2*imgH,imgW);
      else
         % normalize spin image
         spinImg = spinImg./nPts;
      end
      spinImgs(:,:,i) = spinImg;
   else
%       fprintf('#\n%i not enough neighbors!              \n',i);
   end

end


% clear the kd tree
kdtree([],[],root);



