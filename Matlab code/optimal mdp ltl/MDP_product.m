function P=MDP_product(M,R)
P.state_no=M.state_no*R.state_no;
P.S=reshape(1:P.state_no,M.state_no,R.state_no);
P.control_no=M.control_no;

P.S0=P.S(M.S0,R.S0);

P.P=cell(1,P.control_no);
for uu=1:P.control_no
    P.P{uu}=sparse(P.state_no,P.state_no);
    prob=M.P{uu};
    [states,nextstates]=find(prob);
    
    for ii=1:length(states)
        state=states(ii);
        nextstate=nextstates(ii);
        for Rstate=1:R.state_no
            % If no observation, M.obs=0 in this case it is set to 2^N_p
            if M.obs(state)==0
                nextRstate=R.trans(Rstate, 2^R.N_p);
                P.P{uu}(P.S(state,Rstate),P.S(nextstate,nextRstate))...
                    =prob(state,nextstate);
            else
                nextRstate=R.trans(Rstate, M.obs(state));
                P.P{uu}(P.S(state,Rstate),P.S(nextstate,nextRstate))...
                    =prob(state,nextstate);
            end
        end
    end        
end

P.C=cell(1,P.state_no);
for ii=1:P.state_no % control
    [mstate,rstate]=find(P.S==ii);
    P.C{ii}=M.C{mstate};
end

if ~verifyMDPprob(P)
    error('product MDP failed the MDP test');
else
    disp('product MDP passed prob. test');
end

% Generate Accepting state sets L and K
for pair=1:R.pairs
    P.K{pair}=[];
    P.L{pair}=[];
    for state=1:P.state_no
        [mstate,rstate]=find(P.S==state);
        if ~isempty(find(R.K{pair}==rstate))
            P.K{pair}=[P.K{pair},state];
        elseif ~isempty(find(R.L{pair}==rstate))
            P.L{pair}=[P.L{pair},state];
        end
    end
end