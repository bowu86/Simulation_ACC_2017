function pass=verifyMDPprob(M)
pass=false;
tol = 1e-8;

% Check prob
for jj=1:M.control_no
    prob=M.P{jj};
    for ii=1:M.state_no
        if find(M.C{ii}==jj)
            if abs(sum(prob(ii,:)) - 1) > tol
                fprintf('MDP prob. check failed for state %i and control %i\n',ii,jj);
                prob(ii,:)
                return
            end
        end
    end
    
    if ~isempty(find(M.P{jj} > M.state_no))
        fprintf('MDP for action %i contains states outside of M.state_no', jj);
        return
    end
end

% Check actions, every state need to have at least one action
if find(cellfun('isempty',M.C))
    find(cellfun('isempty',M.C))
    disp('Above states have no action');
    return
end
    

pass=true;
%disp('MDP prob check passed');