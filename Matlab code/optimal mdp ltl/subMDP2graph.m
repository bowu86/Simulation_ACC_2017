function G=subMDP2graph(subMDP,Pn)
% Generate di-graph induced by a sub-MDP
% structure subMDP has field states (the state map from Pn), state_no, control_no
% and C

% The below code is for debugging/testing
% Pn.states=1:Pn.state_no;
% subMDP=Pn;
% Above code are for debug use


G.SA_no=subMDP.state_no*subMDP.control_no;
G.SAmap=reshape(1:G.SA_no,subMDP.state_no,subMDP.control_no);

G.state_no=subMDP.state_no+G.SA_no;
G.adj=sparse(G.state_no,G.state_no);

for Gstate=1:G.state_no
    if Gstate<=subMDP.state_no
        % Gstate=A state of subMDP
        G.adj(Gstate,subMDP.state_no+G.SAmap(Gstate,subMDP.C{Gstate}))=1;
    else
        % Gstate=A state-action pair
        SAstate=Gstate-subMDP.state_no;
        
        % THE FIND BELOW CAN BE OPTIMIZED!!!
        [Mstate,Astate]=find(G.SAmap==SAstate);
        % TODO: OPTIMIZE THE ABOVE LINE using floor like below
        %Astate=floor(SAstate/subMDP.state_no)+1;
        
        if ~isempty(find(subMDP.C{Mstate}==Astate))
            statePn=subMDP.states(Mstate);
            postPn=find(Pn.P{Astate}(statePn,:));
            for postStatePn=postPn
                postStateSubMDP=find(subMDP.states==postStatePn);
                if isempty(postStateSubMDP)
                    postStateSubMDP
                    postStatePn
                    error('SubMDP is not correct, post state not in SubMDP');
                end
                % postStateSubMDP is a state of subMDP
                G.adj(Gstate,postStateSubMDP)=1;
            end
        end
    end
end
% 
% return
% state_action=0;
% for ii=1:subMDP.state_no
%     state_action=state_action+length(subMDP.C{ii});
% end
% 
% G.state_no=subMDP.state_no+state_action;
% 
% 
% 
% % G.states are the states of G associated w/ state-action pairs
% G.states=cell(1,state_action);
% 
% ind=1;
% for ii=1:subMDP.state_no
%     for jj=1:length(subMDP.C{ii})
%         G.states{ind}=[subMDP.S(ii),subMDP.C{ii}(jj)];
%         ind=ind+1;
%     end    
% end
% 
% if ind-1~=state_action
%     error('state-action pair problem! %i not equal to %i',ind-1,state_action);
% end
% 
% G.adj=sparse(G.state_no,G.state_no);
% for ii=1:subMDP.state_no
%     for jj=1:length(G.states)
%         sa_pair=G.states{jj};
%         if sa_pair(1)==subMDP.S(ii)
%             G.adj(ii,jj+subMDP.state_no)=1;
%         end
%     end
% end
% 
% for ii=1:length(G.states)
%     state=G.states{ii}(1);
%     action=G.states{ii}(2);
%     post=find(Pn.P{action}(state,:));
%     for jj=1:length(post)
%         if ~isempty(find(subMDP.S==post(jj)))            
%             G.adj(ii+subMDP.state_no,subMDP.S==post(jj))=1;
%         else
%             fsprintf('state %i not found in sub-MDP, but in post of state %i with action %i',post(jj),state,action);
%             error('sub-MDP error');
%         end
%     end
% end