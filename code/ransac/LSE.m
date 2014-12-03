function [n_est ro_est X Y Z]=LSE(p)
%© Tim Zaman 2010, input: p (points)
% Works like [n_est ro_est X Y Z]=LSE(p)
% p should be a Mx3; [points x [X Y Z]]</code>
 
%Calculate mean of all points
pbar=mean(p);
for i=1:length(p)
A(:,:,i)=(p(i,:)-pbar)'*(p(i,:)-pbar);
end
 
%Sum up all entries in A
Asum=sum(A,3);
[V ~]=eig(Asum);
 
%Calculate new normal vector
n_est=V(:,1);
 
%Calculate new ro
ro_est=dot(n_est,pbar);

% minx = -2;
% maxx = 2;
% 
% miny = -2;
% maxy = 2;
maxx = max(p(:,1));
minx = min(p(:,1));

maxy = max(p(:,2));
miny = min(p(:,2));

xskip = maxx-minx; xskip = xskip/10;
yskip = maxy-miny; yskip = yskip/10;

[X,Y]=meshgrid(minx:xskip:maxx,miny:yskip:maxy);
Z=(ro_est-n_est(1)*X-n_est(2).*Y)/n_est(3);
end