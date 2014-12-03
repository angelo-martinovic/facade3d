[points,~,~] = ReadPCLFromPly('/esat/sadr/amartino/monge428New/data/pcloud_gt_test_new_GCO.ply');

fprintf('KNN search...');
idx = knnsearch(points,points,'K',7);
fprintf('Done.');

n=size(idx,1);
ii=[];jj=[];ss=[];
for i=2:7
    ii=[ii; 1:n]; 
    jj=[jj; idx(:,i)];
    ss=[ss; ones(n,1)];
end

W = sparse(ii,jj,ss);

D= perform_dijkstra_fast(W,1:n);