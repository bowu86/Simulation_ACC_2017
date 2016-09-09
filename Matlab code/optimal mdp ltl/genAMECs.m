function [allAMECstates, PAMECs, ActAMECs] = genAMECs(P)

allAMECstates=[];
% Generate MDP w/o L (Pn) for each pair
% State space shrinked by L state
% Pn.states store indices of states in Pn corresponding to P

PAMECs={}; ActAMECs={};
allAMECstates = [];
A={};
for pair=1:length(P.K)
    disp(sprintf('Generating MDP w/o L states (Pn) for pair %i', pair));
    K=P.K{pair};
    L=P.L{pair};
    
    % Delete Pn variable if exists
    clear Pn;
    
    % Pn.states maps from index of statePn to index of stateP
    Pn.states=setdiff(1:P.state_no,L);
    
    % Set the accepting states:
    Pn.K=[];
    for Pstate=K
        Pn.K=[Pn.K,find(Pn.states==Pstate)];
    end
    
    % Add a dummy state
    Pn.states=[Pn.states,-1];
    % Add a dummy control
    Pn.control_no=P.control_no+1;
    Pn.state_no=length(Pn.states);
    Pn.S0=find(Pn.states==P.S0);
    
    % Populate action
    Pn.C=cell(1,Pn.state_no);
    for statePn=1:(Pn.state_no-1)
        Pn.C{statePn}=P.C{Pn.states(statePn)};
    end
    % Action for the dummy state
    Pn.C{Pn.state_no}=Pn.control_no;
    
    % Populate prob. fn and remove actions if needed
    Pn.P=cell(1,Pn.control_no);
    for uu=1:Pn.control_no-1
        Pn.P{uu}=sparse(Pn.state_no,Pn.state_no);
        for statePn=1:(Pn.state_no-1)
            if ~isempty(find(Pn.C{statePn}==uu))
                % Generate post of the coresponding state in P
                % post now is states in P
                post=find(P.P{uu}(Pn.states(statePn),:));
                % map post to states (indices) of Pn
                postn=[];
                for Pstate=post
                    postn=[postn,find(Pn.states==Pstate)];
                end
                
                % post \subseteq Pn.states <=> setdiff(post,Pn.states)=empty
                if isempty(setdiff(post,Pn.states))
                    % Keep uu as a valid action, and transfer the prob. fn
                    Pn.P{uu}(statePn,postn)=P.P{uu}(Pn.states(statePn),post);
                else
                    % Remove action uu
                    Pn.C{statePn}=setdiff(Pn.C{statePn},uu);
                    %fprintf('Action %i removed from state %i\n',uu,Pn.states(statePn));
                end
            end
        end
    end
    
    % For the dummy state
    Pn.P{Pn.control_no}=sparse(Pn.state_no,Pn.state_no);
    % For states w/ no actions, make it transfer to the dummy state w/ prob. 1
    blockstates=[];
    for statePn=1:(Pn.state_no-1)
        if isempty(Pn.C{statePn})
            Pn.P{Pn.control_no}(statePn,Pn.state_no)=1;
            Pn.C{statePn}=Pn.control_no;
            blockstates=[blockstates, statePn];
        end
    end
    % Dummy state self loop w/ prob. 1
    Pn.P{Pn.control_no}(Pn.state_no,Pn.state_no)=1;
    
    if ~verifyMDPprob(Pn)
        error(sprintf('Pn for pair %i failed the MDP test', pair));
    else
        disp(sprintf('Pn for pair %i passed prob. test', pair));
    end
    
    % Generate MECs
    %disp('Generating the MECs and AMECs');
    A=Pn.C; %A(s), s\in Pn.S
    MECs={};
    MECs_new={1:Pn.state_no};
    itercount=0;
    while ~isequal_cell(MECs,MECs_new)
        itercount=itercount+1;
        %disp('===========================================');
        %fprintf('New iteration, count=%i\n', itercount);
        MECs=MECs_new;
        MECs_new={};
        for ii=1:length(MECs)
            %fprintf('Current Sub-MDP: # of states %i\n',length(MECs{ii}));
            Rstates=[];
            MEC=MECs{ii};
            
            % MEC is a subMDP
            % subMDP.states maps from index of statesubMDP to index of statePn
            subMDP.states=MEC;
            subMDP.state_no=length(subMDP.states);
            subMDP.C=cell(1,subMDP.state_no);
            subMDP.control_no=Pn.control_no;
            for subMDPstate=1:subMDP.state_no
                subMDP.C{subMDPstate}=A{subMDP.states(subMDPstate)};
            end
            G=subMDP2graph(subMDP,Pn);
            %disp('Induced graph for sub-MDP generated');
            
            fprintf('Computing connected components... size of G:%d\n', G.state_no);
            [S,C]=graphconncomp(G.adj);
            disp('SCCs computed.');
            
            % Find non-trivial SCCs
            SCCs={};
            for SCC=1:S
                % For each SCC candidate
                SCCstates=find(C==SCC);
                if isempty(SCCstates)
                    error('error in SCCs!, C can not be found');
                end
                
                if length(SCCstates)>1
                    % Found non-trivial SCC
                    % states are indices of G, need to translate to indices of
                    % subMDP
                    subMDPstates=SCCstates(SCCstates<=subMDP.state_no);
                    if isempty(subMDPstates)
                        error('subMDP state is empty');
                    end
                    
                    % Need to translate states from index of subMDP to states
                    % of Pn
                    SCCs{end+1}=subMDP.states(subMDPstates);
                    %fprintf('Found non-trivial SCC: %s\n',mat2str(SCCs{end}));
                end
            end
            %fprintf('Total number of non-trivial SCCs: %i\n',length(SCCs));
            
            for jj=1:length(SCCs)
                states=SCCs{jj};
                for state=states
                    for uu=A{state}
                        % post=find(Pn.P{uu}(state,:))
                        % if post \subseteq states
                        % then setdiff(post,states)=empty
                        
                        %disp(sprintf('operating on action %i',uu));
                        %disp(['post= ',mat2str(find(Pn.P{uu}(state,:)))]);
                        %disp(['states= ',mat2str(states)]);
                        if ~isempty(setdiff(find(Pn.P{uu}(state,:)),states))
                            % remove u from A(s)
                            A{state}=setdiff(A{state},uu);
                            %disp(sprintf('Action %i removed from state %i',uu,state));
                            if isempty(A{state})
                                %fprintf('State %i has no action, queued to be pruned\n',state);
                                Rstates=[Rstates,state];
                            end
                        end
                    end
                end
            end
            
            while ~isempty(Rstates)
                %disp(['Pruning states in Rstates... Rstates=',mat2str(Rstates)]);
                state=Rstates(1);
                Rstates=setdiff(Rstates,state);
                %subMDP.states=setdiff(subMDP.states,state);
                %fprintf('Pruning state %i\n',state);
                MEC=setdiff(MEC,state);
                
                % Pruning states in MEC going to state
                for uu=1:Pn.control_no
                    pre=intersect(MEC,find(Pn.P{uu}(:,state))');
                    for prestate=pre
                        if ~isempty(find(A{prestate}==uu))
                            % find uu actions of prestate
                            % remove uu
                            A{prestate}=setdiff(A{prestate},uu);
                            if isempty(A{prestate})
                                %fprintf('State %i has no action, queued to be pruned\n',prestate);
                                Rstates=[Rstates,prestate];
                            end
                        end
                    end
                end
            end
            
            % Generating new set of subMDPs
            for jj=1:length(SCCs)
                SCC=SCCs{jj};
                if ~isempty(intersect(MEC,SCC))
                    MECs_new{end+1}=sort(intersect(MEC,SCC));
                    %disp('New sub-MDP')
                    %fprintf('states: %s\n',mat2str(MECs_new{end}));
                else
                    %fprintf('SCC %i is removed (no intersection)\n',jj);
                end
            end
        end
    end
    
    AMECs={};
    for ii=1:length(MECs)
        if ~isempty(intersect(MECs{ii},Pn.K))
            %disp(['Found accepting MEC, states= ',mat2str(MECs{ii})]);
            AMECs{end+1}=MECs{ii};
        end
    end
    
    %% Convert AMECs from Pn to P, add to AMEC states, convert A for Pn to
    %% A for P
    PAMECs{pair} = {};
    for ii=1:length(AMECs)
        % Make sure no dummy state in AMEC
        if isempty(find(AMECs{ii}==Pn.state_no))
            PAMECs{pair}{end+1} = Pn.states(AMECs{ii});
        end
    end
    
    ActAMECs{pair} = {};
    if ~isempty(PAMECs{pair})
        ActAMECs{pair} = cell(1,P.state_no);
        for statePn = 1:(Pn.state_no-1)
            ActAMECs{pair}{Pn.states(statePn)}=A{statePn};
        end
    end
    
    for ii=1:length(AMECs)
        % Make sure no dummy state in AMEC
        if isempty(find(AMECs{ii}==Pn.state_no))
            allAMECstates=[allAMECstates, Pn.states(AMECs{ii})];
        end
    end
end

allAMECstates = unique(allAMECstates);