function plotSamplePaths(T, M, spaths)
% Plot sample paths

if ~isempty(spaths) && iscell(spaths)
    for ii = 1:length(spaths)
        spath = spaths{ii};
        
        CMstate = spath(1);
        CVstate = M.S(CMstate,1);
        hold on
        plot(T.nodes(CVstate).position(1), T.nodes(CVstate).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'green');
        for jj = 1:(length(spath)-1)
            CMstate = spath(jj); NMstate = spath(jj+1);
            CVstate = M.S(CMstate,1); NVstate = M.S(NMstate,1);
            hold on
            plot([T.nodes(CVstate).position(1),T.nodes(NVstate).position(1)], [T.nodes(CVstate).position(2),T.nodes(NVstate).position(2)], 'k-', 'LineWidth', 3);
        end
    end
end