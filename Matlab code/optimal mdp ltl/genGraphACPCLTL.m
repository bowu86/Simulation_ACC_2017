function T=genGraphACPCLTL()
% return T: TS, N: all states
T.box_state_no=11*16;
% number of rows
T.numRow=16;
T.numCol=T.box_state_no / T.numRow;

% Define regions that should be empty
R = [];
r.exclude.x = 3:6;
r.exclude.y = 3:7;
% r.exception.x = 3;
% r.exception.y = 5;
R = [R, r];

r.exclude.x = 3:6;
r.exclude.y = 10:12;
% r.exception.x = 3;
% r.exception.y = 11;
R = [R, r];

r.exclude.x = 8:10;
r.exclude.y = 5:11;
% r.exception.x = 8;
% r.exception.y = 8;
R = [R, r];

% Create Nodes
T.nodes=[];
for ii=1:T.box_state_no
    ypos = floor(ii/T.numCol);
    xpos = mod(ii, T.numCol) - 1;
    if mod(ii, T.numCol) == 0
        ypos = ypos - 1;
        xpos = T.numCol - 1;
    end
    
    xpos = xpos + 1;
    ypos = ypos + 1;
    
    exclude = false;
    for rr = 1:length(R)
        r = R(rr);
        if ismember(xpos, r.exclude.x) && ismember(ypos, r.exclude.y)
            if isfield(r, 'exception') && ~(r.exception.x == xpos && r.exception.y == ypos)
                continue
            end
            exclude = true;
            break
        end
    end
    
    if ~exclude
        xpos = xpos*10;
        ypos = ypos*10;
        
        T.nodes=[T.nodes, struct('position',[xpos, ypos],'atomicProp',[])];
    end

end

T.state_no = length(T.nodes);
T.control_no = 4;
T.controlMat = sparse(T.state_no, T.control_no);
T.controlCost = sparse(T.state_no, T.control_no);
T.adj = sparse(T.state_no, T.state_no);

for ii=1:T.state_no    
    % Assign neighbour based on proximity
    % Assign control transitions
    ProxDiskR=13;
    for jj=1:T.state_no
        if jj~=ii && norm(T.nodes(ii).position-T.nodes(jj).position)<=ProxDiskR            
            % Control encoding:
            %   2
            % 1 i 3
            %   4
            if T.nodes(ii).position(1) == T.nodes(jj).position(1)
                if T.nodes(ii).position(2) > T.nodes(jj).position(2)
                    T.controlCost(ii,4) = norm(T.nodes(ii).position - T.nodes(jj).position);
                    T.controlMat(ii,4) = jj;
                    T.adj(ii,jj) = 1;
                elseif T.nodes(ii).position(2) < T.nodes(jj).position(2)
                    T.controlCost(ii,2) = norm(T.nodes(ii).position - T.nodes(jj).position);
                    T.controlMat(ii,2) = jj;
                    T.adj(ii,jj) = 1;
                else
                    error('No selfloop allowed for this graph');
                end
            elseif T.nodes(ii).position(2) == T.nodes(jj).position(2)
                if T.nodes(ii).position(1) > T.nodes(jj).position(1)
                    T.controlCost(ii,1) = norm(T.nodes(ii).position - T.nodes(jj).position);
                    T.controlMat(ii,1) = jj;
                    T.adj(ii,jj) = 1;
                elseif T.nodes(ii).position(1) < T.nodes(jj).position(1)
                    T.controlCost(ii,3) = norm(T.nodes(ii).position - T.nodes(jj).position);
                    T.controlMat(ii,3) = jj;
                    T.adj(ii,jj) = 1;
                else
                    error('No selfloop allowed for this graph');
                end
            else
                error('States on graph not aligned on a grid?');
            end
        end
    end
    
    
    
    % Put in observations
%     if isempty(T.nodes(ii).atomicProp)
%         T.obs(ii)=2^T.N_p;
%         continue
%     end
%     ap_obs=[];
%     for jj=1:length(T.nodes(ii).atomicProp)
%         switch T.nodes(ii).atomicProp(jj)
%             case 'a'
%                 ap_obs=[ap_obs, 4];
%             case 'b'
%                 ap_obs=[ap_obs, 3];
%             case 'c'
%                 ap_obs=[ap_obs, 2];
%             case 'd'
%                 ap_obs=[ap_obs, 1];
%                 %case 'e'
%                 %ap_obs=[ap_obs, 5];
%             otherwise
%                 disp(T.nodes(ii).atomicProp);
%                 error('Atomic Proposition not expected!');
%         end
%         ap_obs=sort(ap_obs);
%     end
%     alph_ind=zeros(1,T.N_p);
%     alph_ind(ap_obs)=1;
%     T.obs(ii)=bin2dec(char(alph_ind+double('0')));
end

% Assign initial state of T
T.Q0 = 1;

%% Assign probabilities
% Prob. of Picking up
%T.centroid = [T.numCol * 10 /2, T.numRow * 10 /2];
T.fp1 = [0, 91]; T.fp2 = [91, 0];
T.fps = [T.fp1, T.fp2];
% for ii=1:T.state_no
%     T.nodes(ii).probPU = 1 - norm(T.nodes(ii).position - T.centroid) / 100;
%     if T.nodes(ii).probPU > 1 || T.nodes(ii).probPU < 0
%         error('prob. out or range');
%     end
% end
for ii=1:T.state_no
    d1 = norm(T.nodes(ii).position - T.fp1); d2 = norm(T.nodes(ii).position - T.fp2); 
    T.nodes(ii).probPU = exp(-d1/10) + exp(-d2/10);
    if T.nodes(ii).probPU > 1
        warning('T.nodes(%i).probPU is greater than 1, clipping', ii);
        T.nodes(ii).probPU = 1;
    end
    
    if T.nodes(ii).probPU > 1 || T.nodes(ii).probPU < 0
        error('prob. out or range');
    end
end

% Prob. of DROPOFFA if picked up
maxNorm = max(arrayfun(@(x) norm(x.position), T.nodes));
for ii=1:T.state_no
    T.nodes(ii).probGOTOA = norm(T.nodes(ii).position) / maxNorm;
    if T.nodes(ii).probGOTOA > 1 || T.nodes(ii).probGOTOA < 0
        error('prob. out or range @ %d', ii);
    end
end

% Set DO proposition
T.DOA = 1;
T.DOB = T.state_no;

fprintf('T constructed, # of states: %i, initial state of T is %i\n',T.state_no, T.Q0);

