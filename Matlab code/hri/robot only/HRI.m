%Minimize average cost per cycle
close all
clc
clear variables
tic
Task.state_no=6;% number of states in task
Fatigue.state_no=3;% number of states in fatigue model
Robot.state_no=2; % number of states in robot model
Trust.state_no=3; % number of states in trust model
%% Construct the MDP
Transition=parse_transition_ltl_robot();%Transition(from,to,act)=p(from,act,to)
M.state_no = Task.state_no*Fatigue.state_no*Robot.state_no*Trust.state_no;
M.S = states_robot(); 
%Mstate = (Task,Robot,Fatigue,Trust) starting from [0,0,0,0]
M.S0 = 1; 
% Set of controls
M.control_no = 6;
M.Act = 1:M.control_no;
% 'a0r','a1r','a2r','a3r','a0h','a1h','a2h','a3h','reset','repair'
M.CONTROLSTRING{1} = 'a0r'; M.CONTROLSTRING{3} = 'a2r'; 
M.CONTROLSTRING{2} = 'a1r'; M.CONTROLSTRING{4} = 'a3r';
M.CONTROLSTRING{5} = 'reset'; M.CONTROLSTRING{6} = 'repair';
% Enabled controls at each state
M.C=cell(1,M.state_no);
for Mstate = 1:M.state_no
    M.C{Mstate} = [];
    for a =1:M.control_no
        if sum(Transition(Mstate,:,a))>0 % action a is defined on Mstate
            M.C{Mstate} = [M.C{Mstate}, a];
        end
        
    end
end

% Transition Prob. Matrices
M.P=cell(1, M.control_no);
for a = 1:M.control_no
    M.P{a} = sparse(M.state_no,M.state_no);
end

for Mstate = 1:M.state_no
            for uu = M.C{Mstate}
                for NextMstate=1:M.state_no
                    M.P{uu}(Mstate, NextMstate) = Transition(Mstate,NextMstate,uu);
                end
            end              
 
end

% Set APs
M.FINISH = []; M.OK = [];M.MAL=[];
for Mstate = 1:M.state_no
    if M.S(Mstate,1)==5% Task is at state 5, that is, the finishing state
        M.FINISH=[M.FINISH,Mstate];
    end
    if M.S(Mstate,2)==0% robot is at normal state
        M.OK=[M.OK,Mstate];
    end
    if M.S(Mstate,2)==1% robot is at malfunction state
        M.MAL=[M.MAL,Mstate];
    end
 
end
    
if ~verifyMDPprob(M)
    error('M failed the MDP test');
else
    disp('M passed the MDP test');
end

%% Generate DRA and assign M.obs
casestudy = 'ACC1'; % Change this to your formula of choice
N_p = 3;
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
%% Define control cost and and P.Spi
P.g = sparse(P.state_no, P.control_no);
P.Spi = [];
Cost=cost_assignment_robot();%Cost(s,a)
for Pstate = 1:P.state_no
    [Mstate,Astate] = find(P.S==Pstate);
    for uu = P.C{Pstate}        
        ucost = Cost(Mstate,uu);
        if ucost <= 0
            error('At state %d, control %d has cost <=0', Pstate, uu);
        end
        P.g(Pstate, uu) = ucost;
    end
    
    if ismember(Mstate, M.FINISH)
        P.Spi = [P.Spi, Pstate];
    end
end

% Option to load AMECs from previous calculation
loadamecs = false;

if loadamecs
    disp('Loading existing mat file from AMECs.mat');
    tempload = load('AMEC_HRI.mat');
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


fsize=30;
figure
plot(JJ(1,:),'LineWidth',4)
title('Cost from the initial state vs. Iteration', 'FontSize', fsize)
xlabel('Iteration #', 'FontSize', fsize)
ylabel('$J^\star(s_{\mathcal P0})$', 'Interpreter', 'latex','FontSize',fsize)
set(gca,'fontsize',fsize)
axis tight
policy=cell(M.state_no,1);
for i=1:Pstate
    idx=round((mod(i,M.state_no)));
    if idx==0
        idx=M.state_no;
    end
    if A_opt(i)~=0
        policy{idx}=[policy{idx},' ',M.CONTROLSTRING{A_opt(i)}];
    end
end
toc
