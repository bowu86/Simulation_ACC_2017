function [spath, upath, ACPC] = genSamplePath(M, mu, H)
% Generate a sampth path based on stationary policy mu up to stage H as the
% horizon

currentState = M.S0;
spath = currentState;
upath = [];
totalCost = 0; totalCycles = 0;
for ii = 1:H
    uu = mu(currentState);
    upath = [upath, uu];
    totalCost = totalCost + M.g(currentState, uu);
    if ~isempty(find(M.Spi==currentState, 1));
        totalCycles = totalCycles + 1;
    end
    
    [~,Nstates,probDist] = find(M.P{uu}(currentState,:));
    if length(Nstates) > 1
        currentState = randsample(Nstates, 1, true, probDist);
    else
        currentState = Nstates;
    end
    
    spath = [spath, currentState];
end

ACPC = totalCost / totalCycles;