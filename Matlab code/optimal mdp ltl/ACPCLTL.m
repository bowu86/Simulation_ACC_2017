close all
clc
clear variables
tic

% Color constants
redColor = [1 0 0];
blueColor = [0 0 1];
yellowColor = [1 1 0];
greenColor = [0 1 0];
blackColor = [0 0 0];
whiteColor = [1 1 1];
magentaColor = [1 0 1]; 

%% Construct the graph
reconstruct = 1;
if ~reconstruct && exist('ACPCLTLT.mat', 'file')
    load ACPCLTLT.mat;        
else
    T=genGraphACPCLTL();
end
plotGraph(T, {'probPU', 'probGOTOA'},{[blackColor; whiteColor], [blackColor; whiteColor]});

%% Construct the MDP
M.state_no = T.state_no * 3;
M.S = cartesian_product(1:T.state_no, [0 1 2]);

Vstate = T.state_no-T.numCol+1;
GTstate = 0;
M.S0 = Vstate*3 + (GTstate-2); 
% To translate from Mstate -> (Vstate, GTstate), do Vstate = M.S(Mstate,1); GTstate = M.S(Mstate,2); 
% GTstate = 1 -> GOTOA, GTstate = 2 -> GOTOB, GTstate = 0 -> neither
% for the other direction, do Mstate = Vstate*3 + (GTstate-2);

% Set of controls
% Control encoding:
%   2
% 1 i 3
%   4
M.control_no = T.control_no * 2;
M.Act = cartesian_product(1:T.control_no, [0 1]);
% To translate from uu -> (Control, PU), do Control = M.Act(uu,1); PU = M.Act(uu,2);, for the
% other direction, do uu = Control*2 + (PU-1);
M.CONTROLSTRING{1} = 'LEFT, NO PU'; M.CONTROLSTRING{3} = 'UP, NO PU'; M.CONTROLSTRING{5} = 'RIGHT, NO PU'; M.CONTROLSTRING{7} = 'DOWN, NO PU';
M.CONTROLSTRING{2} = 'LEFT, PU'; M.CONTROLSTRING{4} = 'UP, PU'; M.CONTROLSTRING{6} = 'RIGHT, PU'; M.CONTROLSTRING{8} = 'DOWN, PU'; 

% Unabled controls at a state
M.C=cell(1,M.state_no);
for Mstate = 1:M.state_no
    M.C{Mstate} = [];
    
    Vstate = M.S(Mstate,1); GTstate = M.S(Mstate,2);
    switch GTstate
        case 0            
            for uu = 1:M.control_no
                Control = M.Act(uu,1); PU = M.Act(uu,2);
                if T.controlMat(Vstate,Control)
                    M.C{Mstate} = [M.C{Mstate}, uu];
                end
            end
        case {1, 2}
            for uu = 1:M.control_no
                Control = M.Act(uu,1); PU = M.Act(uu,2);
                if T.controlMat(Vstate,Control) && ~PU
                    M.C{Mstate} = [M.C{Mstate}, uu];
                end
            end
        otherwise
            error('out of bound');
    end
end

% Transition Prob. Matrices
M.P=cell(1, M.control_no);
for uu = 1:M.control_no
    M.P{uu} = sparse(M.state_no,M.state_no);
end

for Mstate = 1:M.state_no
    Vstate = M.S(Mstate,1); GTstate = M.S(Mstate,2);
    switch GTstate
        case 0 
            for uu = M.C{Mstate}
                Control = M.Act(uu,1); PU = M.Act(uu,2);
                if PU
                    probPU = T.nodes(Vstate).probPU;
                    probGOTOA = T.nodes(Vstate).probGOTOA;
                    
                    % Transition to (Vnext,0) is 1-probPU
                    NextVstate = T.controlMat(Vstate,Control); NextGTstate = 0;
                    NextMstate = NextVstate*3 + (NextGTstate-2);
                    M.P{uu}(Mstate, NextMstate) = 1-probPU;
                    
                    % Transition to (Vnext,1) is probPU * probGOTOA
                    NextVstate = T.controlMat(Vstate,Control); NextGTstate = 1;
                    NextMstate = NextVstate*3 + (NextGTstate-2);
                    M.P{uu}(Mstate, NextMstate) = probPU * probGOTOA;
                    
                    % Transition to (Vnext,2) is probPU * (1-probGOTOA)
                    NextVstate = T.controlMat(Vstate,Control); NextGTstate = 2;
                    NextMstate = NextVstate*3 + (NextGTstate-2);
                    M.P{uu}(Mstate, NextMstate) = probPU * (1-probGOTOA);
                else
                    NextVstate = T.controlMat(Vstate,Control); NextGTstate = 0;
                    NextMstate = NextVstate*3 + (NextGTstate-2);
                    M.P{uu}(Mstate, NextMstate) = 1;
                end
            end
        case {1, 2}
            for uu = M.C{Mstate}
                Control = M.Act(uu,1); PU = M.Act(uu,2);
                if PU
                    error('PU at this state cannot be 1');
                end                
                NextVstate = T.controlMat(Vstate,Control); NextGTstate = 0;
                NextMstate = NextVstate*3 + (NextGTstate-2);
                M.P{uu}(Mstate, NextMstate) = 1;                
            end
        otherwise
            error('out of bound');                
    end
end

% Set APs
M.PICKUP = []; M.DROPOFFA = []; M.DROPOFFB = []; M.GOTOA = [];
for Mstate = 1:M.state_no
    Vstate = M.S(Mstate,1); GTstate = M.S(Mstate,2);
    if GTstate == 1
        M.PICKUP = [M.PICKUP, Mstate];
        M.GOTOA = [M.GOTOA, Mstate];
    elseif GTstate == 2
        M.PICKUP = [M.PICKUP, Mstate];
    end
    
    if ismember(Vstate, T.DOA) && GTstate == 0
        M.DROPOFFA = [M.DROPOFFA, Mstate];
    end
    if ismember(Vstate, T.DOB) && GTstate == 0
        M.DROPOFFB = [M.DROPOFFB, Mstate];
    end
end
    
if ~verifyMDPprob(M)
    error('M failed the MDP test');
else
    disp('M passed the MDP test');
end

%% Generate DRA and assign M.obs
casestudy = 'TAC'; % Change this to your formula of choice
N_p = 4;

% Assume that either formula in text form (.ltl file) or in DRA form (.out file created by ltl2dstar)
% is available for the LTL input
if exist(['./' casestudy '.out'], 'file')
    R=create_DRA(['./' casestudy '.out'],N_p);
elseif exist(['./' casestudy '.ltl'], 'file')
    if isunix
        dos(['./bin/ltl2dstar --ltl2nba=spin:./bin/ltl2ba --output=automaton ./' casestudy '.ltl ./' casestudy '.out']);
    elseif ispc
        dos(['./bin/ltl2dstar.exe --ltl2nba=spin:./bin/ltl2ba.exe --output=automaton ./' casestudy '.ltl ./' casestudy '.out']);
    else
        error('Unknown or unsupported operating system....');
    end
    
    if exist(['./' casestudy '.out'], 'file')
        R=create_DRA(['./' casestudy '.out'],N_p);
    else
        error('Cannot create DRA output with ltl2dstar, possible cause: input file does not exist or ltl2dstar binary does not exist');
    end
else
    error('Input LTL formula not found');
end

% Assign Observations
M.obs=sparse(1,M.state_no);
for state=1:M.state_no
    ap_obs=[];
    
    for ii=1:length(R.APs)
        ap_pos=length(R.APs)+1-ii;
        if find(getfield(M, R.APs{ii})==state)
            ap_obs=[ap_obs, ap_pos];
        end
    end
    
    if ~isempty(ap_obs)
        alph_ind=zeros(1,length(R.APs));
        alph_ind(ap_obs)=1;
        M.obs(state)=bin2dec(char(alph_ind+double('0')));
    end
end

%% Generate product MDP and AMECs and Find Optimal policy going to AMEC set
disp('Generating Product MDP');
P=MDP_product(M,R);

% Option to load AMECs from previous calculation
loadamecs = true;

if loadamecs
    disp('Loading existing mat file from AMECs.mat');
    tempload = load('AMECs.mat');
    allAMECstates = tempload.allAMECstates;
    AMECs = tempload.AMECs;
    A = tempload.A;
    x = tempload.x;
    A_opt = tempload.A_opt;
    clear tempload
else
    disp('Obtaining AMECs from scratch...');
    [allAMECstates, AMECs, A] = genAMECs(P);    
    
    disp('Solving MRP...');
    [x,A_opt]=MRP(P,allAMECstates,[]);
    fprintf('Max prob. of satisfying spec from initial state=%1.3f\n',x(P.S0));    
    unreachablestates=find(x==0);
    
    save(['AMECs', date, '.mat']);
end

%% Define control cost and and P.Spi
P.g = sparse(P.state_no, P.control_no);
P.Spi = [];
for Pstate = 1:P.state_no
    [Mstate,Astate] = find(P.S==Pstate);
    Vstate = M.S(Mstate,1); GTstate = M.S(Mstate,2);
    for uu = P.C{Pstate}
        Control = M.Act(uu,1); PU = M.Act(uu,2);        
        ucost = T.controlCost(Vstate, Control);
        if ucost <= 0
            error('At state %d, control %d has cost <=0', Pstate, uu);
        end
        P.g(Pstate, uu) = ucost;
    end
    
    if ismember(Mstate, M.PICKUP)
        P.Spi = [P.Spi, Pstate];
    end
end

%% Generate subMDP and compute ACPC cost for each subMDP
% FIXME: GENERALIZE TO MULTIPLE PAIRS
for pair = 1:1
    Act = A{pair};
    for amecind = 1:length(AMECs{pair})
        % AMECs{pair}{amecind} maps from state in subMDP to P
        M_AMEC.S = AMECs{pair}{amecind};
        M_AMEC.state_no = length(M_AMEC.S);
        M_AMEC.control_no = P.control_no;
        M_AMEC.CONTROLSTRING = M.CONTROLSTRING;
        M_AMEC.P = cell(1, M_AMEC.control_no);
        for uu=1:M_AMEC.control_no
            M_AMEC.P{uu}=sparse(M_AMEC.state_no,M_AMEC.state_no);
        end
        M_AMEC.Spi = [];
        M_AMEC.g = sparse(M_AMEC.state_no, M_AMEC.control_no);
        
        % Construct the subMDP
        for M_AMECstate = 1:M_AMEC.state_no
            M_AMEC.C{M_AMECstate} = Act{M_AMEC.S(M_AMECstate)};
            for uu = M_AMEC.C{M_AMECstate}
                for NextPstate = find(P.P{uu}(M_AMEC.S(M_AMECstate),:))
                    NextM_AMECstate = find(M_AMEC.S == NextPstate);
                    if isempty(NextM_AMECstate)
                        error('SubMDP construction error, AMEC possibly wrong');
                    end
                    M_AMEC.P{uu}(M_AMECstate, NextM_AMECstate) = P.P{uu}(M_AMEC.S(M_AMECstate), M_AMEC.S(NextM_AMECstate));
                end
                
                M_AMEC.g(M_AMECstate, uu) = P.g(M_AMEC.S(M_AMECstate), uu);
            end
            
            if ismember(M_AMEC.S(M_AMECstate), P.Spi)
                M_AMEC.Spi = [M_AMEC.Spi, M_AMECstate];
            end
        end
        
        % Verify if subMDP is valid
        if ~verifyMDPprob(M_AMEC)
            error('SubMDP ind %i for pair %i failed the MDP test', amecind, pair);
        else
            fprintf('SubMDP ind %i for pair %i passed MDP test\n', amecind, pair);
        end
        
        % Compute ACPC cost and opt. action
        [JJ, mu] = computeACPC(M_AMEC);
        
        % Map the optimal action back from M_AMEC to P for A_opt
        for M_AMECstate = 1:M_AMEC.state_no
            A_opt(M_AMEC.S(M_AMECstate)) = mu(M_AMECstate);
        end
    end
end

%% Check for optimality condition
% for ii = 1:M.state_no
%     %fprintf('Checking opt. condition for state %i\n', ii);
%     pmuii=M.g(ii, mu(ii));
%     nind=find(M.P{mu(ii)}(ii,:));
%     for jj=nind
%         pmuii=pmuii+M.P{mu(ii)}(ii,jj)*hmu(jj);
%         % See if jj is *not* in Spi
%         if isempty(find(M.Spi==jj))
%             pmuii=pmuii+M.P{mu(ii)}(ii,jj)*Jmu(jj);
%         end
%     end
%     pmin=pmuii;
%     umin=mu(ii);
%             
%     for uu=setdiff(M.C{ii},mu(ii))
%         ptemp=M.g(ii, uu);
%         nind=find(M.P{uu}(ii,:));
%         for jj=nind
%             ptemp=ptemp+M.P{uu}(ii,jj)*hmu(jj);
%             % See if jj is *not* in Spi
%             if isempty(find(M.Spi==jj))
%                 ptemp=ptemp+M.P{uu}(ii,jj)*Jmu(jj);
%             end
%         end
%         %disp(sprintf('2nd opt. pass: State %i, control old %s (cost %4.3f) control new %s (cost %4.3f)', ii, CONTROLSTRING{mu(ii)}, pmuii, CONTROLSTRING{uu}, ptemp));
%         if pmin-ptemp>th
%             error('State: %i, control %i with RHS cost %2.3f violate the optimality condition, "opt." u is %i with RHS cost %2.3f', ii, uu, ptemp, umin, pmin);
%         end
%     end    
%         
%     if abs(Jmu(ii)+hmu(ii) - pmin)>th
%         error('State: %i, control %i with RHS cost %2.3f violate the optimality condition, LHS cost is %2.3f', ii, umin, pmin, Jmu(ii)+hmu(ii));
%     end
% end
figure
plot(JJ(1,:))
title('Cost from the initial state vs. Iteration', 'FontSize', 14)
xlabel('Iteration #', 'FontSize', 14)
ylabel('$J^\star(s_{\mathcal P0})$', 'Interpreter', 'latex','FontSize',16)

toc
