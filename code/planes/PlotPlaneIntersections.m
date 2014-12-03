nPlanes = size(filteredPlanes.n,1);
filteredPlanes.d= - sum(filteredPlanes.n.*filteredPlanes.p,2);

cc=filteredPlanes.p;
[pc,score,latent,tsquare] = princomp(cc);
ccProj = score(:,1);
[~,order] = sort(ccProj);

p = points;
figure(101);scatter3(p(:,1),p(:,2),p(:,3));
xlabel('x');ylabel('y');zlabel('z');



separatorLines = struct('x0',[],'a',[]);
for i=1:length(order)-1
    j = i+1;
    
    ii = order(i);
    jj = order(i+1);
       
    n1=filteredPlanes.n(ii,:);
    n2=filteredPlanes.n(jj,:);

    p1=filteredPlanes.d(ii,:);
    p2=filteredPlanes.d(jj,:);



   [x0,a] = PlaneIntersection(n1,n2,p1,p2);

   x1 = x0 + a *10;
   x2 = x0 - a *10;

   line([x2(1) x1(1)],[x2(2) x1(2)],[x2(3) x1(3)]);
           
   separatorLines(i).x0 = x0;
   separatorLines(i).a = a;
    
end