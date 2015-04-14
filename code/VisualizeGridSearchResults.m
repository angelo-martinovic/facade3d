minLeaf=[1 3 10 30 100];
nTrees = [10 20 50 100 200];

oobErrors = zeros(length(minLeaf),length(nTrees));
testTimes = zeros(length(minLeaf),length(nTrees));
testScores = zeros(length(minLeaf),length(nTrees));
for i=1:length(minLeaf)
    for j=1:length(nTrees)
        error=load(sprintf('/usr/data/amartino/Facade3D/output/work/pcl/monge428_3Dclassifier_rf_nTrees_%d_minLeaf_%d_error.mat',nTrees(j),minLeaf(i)));
        oobErrors(i,j)=error.error(end);
        
        testTime=load(sprintf('/usr/data/amartino/Facade3D/output/work/pcl/monge428_3Dclassifier_rf_nTrees_%d_minLeaf_%d_testTime.mat',nTrees(j),minLeaf(i)));
        testTimes(i,j)=testTime.rfTestTime;
        
        testScore=load(sprintf('/usr/data/amartino/Facade3D/output/work/pcl/monge428_3Dclassifier_rf_nTrees_%d_minLeaf_%d_testScore.mat',nTrees(j),minLeaf(i)));
        testScores(i,j) = testScore.score.mean_pascal;
    end
end

%%
figure(2);clf;hold on;
h=surf(nTrees,minLeaf,1-oobErrors,'FaceColor','r');
alpha(h, 0.5);

surf(nTrees,minLeaf,testTimes/1e3,'FaceColor','b');
alpha(h, 0.5);

set(gca,'XScale','log','YScale','log');

xlabel('Number of trees');
ylabel('Min. observ. per leaf');

set(gca,'xtick',nTrees);
set(gca,'XTickLabel',num2str(nTrees'));

set(gca,'ytick',minLeaf);
set(gca,'YTickLabel',num2str(minLeaf'));


fs = 16; 
zlabel({'1-OOB error','time/1e3[s]'},'FontSize',fs); set(gca,'FontSize',round(fs*.8));
set(gcf, 'Color', [1 1 1]); 
grid on;
axis tight;
zlim([0 1])
view(3);
