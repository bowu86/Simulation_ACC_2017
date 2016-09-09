close all
clear variables
clc

% Generate M
P.state_no=20;
P.Spi=[1:16];
P.S0=1;

P.control_no=3;
P.C=cell(1,P.state_no);
% Assigning the set of controls to states
% alpha = 1, beta = 2, gamma = 3
P.C{1}=[2];
P.C{2}=[1,2];
P.C{3}=[1];
P.C{4}=[1];
P.C{5}=[1,2,3];
P.C{6}=[3];
P.C{7}=[1];
P.C{8}=[1,2];
P.C{9}=[1];
P.C{10}=[1,2];
P.C{11}=[1,2];
P.C{12}=[1];
P.C{13}=[1];
P.C{14}=[1,2,3];
P.C{15}=[3];
P.C{16}=[1];
P.C{17}=[1,2];
P.C{18}=[1];
P.C{19}=[1];
P.C{20}=[2];

P.P=cell(1, P.control_no);

% For control alpha:
% index for alpha = 1;
prob=sparse(P.state_no,P.state_no);
prob(2,3)=1;
prob(3,6)=1;
prob(4,1)=1;
prob(5,4)=1;
prob(7,4)=1;
prob(8,7)=1;
prob(9,17)=1;
prob(10,19)=1;
prob(11,12)=1;
prob(12,15)=1;
prob(13,10)=1;
prob(14,13)=1;
prob(16,13)=1;
prob(17,16)=1;
prob(18,17)=1;
prob(19,20)=1;
P.P{1}=prob;

% For control beta:
% index for beta = 2;
prob=sparse(P.state_no,P.state_no);
prob(1,2)=1;
prob(2,3)=0.2;
prob(2,5)=0.8;
prob(5,8)=1;
prob(8,9)=1;
prob(10,11)=1;
prob(11,12)=0.2;
prob(11,14)=0.8;
prob(14,17)=1;
prob(17,18)=1;
prob(20,2)=1;
P.P{2}=prob;

% For control gamma:
% index for gamma = 3;
prob=sparse(P.state_no,P.state_no);
prob(5,6)=0.7;
prob(5,8)=0.3;
prob(6,5)=0.9;
prob(6,9)=0.1;
prob(14,15)=0.7;
prob(14,17)=0.3;
prob(15,14)=0.9;
prob(15,18)=0.1;
P.P{3}=prob;

if ~verifyMDPprob(P)
    error('P failed the MDP test');
else
    disp('P passed prob. test');
end

% Define control cost and string
CONTROLSTRING{1}='alpha';
CONTROLSTRING{2}='beta';
CONTROLSTRING{3}='gamma';
CONTROLCOST(1)=5;
CONTROLCOST(2)=10;
CONTROLCOST(3)=1;

% Simulation Parameter
th=0.001;

iteration=0;

% Find an initial (proper) policy
mu=genProperPolicy(P, P.Spi);

%mu=[2,1,1,1,1,3,1,2,1,1,1,1,1,3,3,1,1,1,1,2];
%mu=[2,1,1,1,1,3,1,1,1,1,2,1,1,1,3,1,1,1,1,2];

while true
    updateMu;
%         updateMu_ACPS;
    iteration=iteration+1;
end

% Check for optimality condition
for ii = 1:P.state_no
    fprintf('Checking opt. condition for state %i\n', ii);
    pmuii=CONTROLCOST(mu(ii));
    nind=find(P.P{mu(ii)}(ii,:));
    for jj=nind
        pmuii=pmuii+P.P{mu(ii)}(ii,jj)*hmu(jj);
        % See if jj is *not* in Spi
        if isempty(find(P.Spi==jj))
            pmuii=pmuii+P.P{mu(ii)}(ii,jj)*Jmu(jj);
        end
    end
    pmin=pmuii;
    umin=mu(ii);
            
    for uu=setdiff(P.C{ii},mu(ii))
        ptemp=CONTROLCOST(uu);
        nind=find(P.P{uu}(ii,:));
        for jj=nind
            ptemp=ptemp+P.P{uu}(ii,jj)*hmu(jj);
            % See if jj is *not* in Spi
            if isempty(find(P.Spi==jj))
                ptemp=ptemp+P.P{uu}(ii,jj)*Jmu(jj);
            end
        end
        %disp(sprintf('2nd opt. pass: State %i, control old %s (cost %4.3f) control new %s (cost %4.3f)', ii, CONTROLSTRING{mu(ii)}, pmuii, CONTROLSTRING{uu}, ptemp));
        if pmin-ptemp>th
            error('State: %i, control %i with RHS cost %2.3f violate the optimality condition, "opt." u is %i with RHS cost %2.3f', ii, uu, ptemp, umin, pmin);
        end
    end    
        
    if abs(Jmu(ii)+hmu(ii) - pmin)>th
        error('State: %i, control %i with RHS cost %2.3f violate the optimality condition, LHS cost is %2.3f', ii, umin, pmin, Jmu(ii)+hmu(ii));
    end
end