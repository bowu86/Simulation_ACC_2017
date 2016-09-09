function mu=genProperPolicy(M, Reach)
% This function takes as input a communicating MDP, and output a proper
% policy with respect to a subset of states Reach
% Pick a random Reach state for constructing the recurrent class

mu = zeros(1,M.state_no);
M.states = 1:M.state_no;
rind = randi(length(Reach));

G = subMDP2graph(M, M);

% Inverse of G.adj reverse its direction, causing graphshortestpath to go
% for a target state instead of a source
[dist, path, pred] = graphshortestpath(G.adj', Reach(rind));

for state = 1:M.state_no
    if state == Reach(rind)
        % If state is source, pick a *random* enabled action
        mu(state) = M.C{state}(randi(length(M.C{state})));
    else        
        nextState = pred(state);
        % nextState is always a S-A state, need to find the A state
        
        % Gstate=A state-action pair
        SAstate=nextState-M.state_no;
        [Mstate,Astate]=find(G.SAmap==SAstate);
        
        if ~isequal(Mstate,state)
            error('Mstate: %i ~= state: %i', Mstate, state);
        end
        mu(Mstate) = Astate;
    end
end