function CurrentInd = plotSamplePathsOnP(T, M, P, spath, InitialInd)
% Plot sample paths

if nargin < 5
    InitialInd = 1;
end

if ~isempty(spath)
    CurrentPstate = spath(InitialInd);
    [CurrentMstate,CurrentAstate] = find(P.S==CurrentPstate);
    CurrentVstate = M.S(CurrentMstate,1);
    hold on
    plot(T.nodes(CurrentVstate).position(1), T.nodes(CurrentVstate).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'g');
    
    pickedup = false;
    for jj = InitialInd:(length(spath)-1)
        CurrentPstate = spath(jj);
        [CurrentMstate,CurrentAstate] = find(P.S==CurrentPstate);
        CurrentVstate = M.S(CurrentMstate,1);
        CurrentInd = jj;
        
        NextPstate = spath(jj+1);
        [NextMstate,NextAstate] = find(P.S==NextPstate);
        NextVstate = M.S(NextMstate,1);
        
        hold on
        plot([T.nodes(CurrentVstate).position(1),T.nodes(NextVstate).position(1)], [T.nodes(CurrentVstate).position(2),T.nodes(NextVstate).position(2)], 'k-', 'LineWidth', 3);
        
        % draw a blue circle if picked up
        if ismember(CurrentMstate, M.PICKUP)
            plot(T.nodes(CurrentVstate).position(1), T.nodes(CurrentVstate).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'b');
            pickedup = true;
        end
        
        % plot until dropped off
        if ismember(CurrentMstate, M.DROPOFFA) || ismember(CurrentMstate, M.DROPOFFB)            
            if pickedup
%                 plot(T.nodes(CurrentVstate).position(1), T.nodes(CurrentVstate).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'g');
                drawnow
                return
            end
        end
        
    end
end