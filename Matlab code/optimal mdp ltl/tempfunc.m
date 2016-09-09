
%% Generate sample paths on P and compute average cycle cost
Nsamples = 1;
Hsamples = 3000;
spaths = {}; ACPCs=[];
for ii = 1:Nsamples
    [spath, upath, ACPC] = genSamplePath(P, A_opt, Hsamples);
    % plot the spath
    
    spaths{length(spaths)+1} = spath;    
    ACPCs = [ACPCs, ACPC];
end

figure;
plotGraph(T);
CurrentInd = plotSamplePathsOnP(T, M, P, spath, CurrentInd);

plot(T.nodes(T.DOA).position(1), T.nodes(T.DOA).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'g');
plot(T.nodes(T.DOB).position(1), T.nodes(T.DOB).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'g');

colorbar('off');
plotSamplePathsOnP(T, M, P, spaths)