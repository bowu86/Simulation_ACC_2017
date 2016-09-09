function [JJ, mu] = computeACPC(M)
%% ACPC
% Simulation Parameter
th=0.001;
iteration=0;
JJ=[];

% Find an initial (proper) policy
mu=genProperPolicy(M, M.Spi);

while true
    disp('======================');
    disp(sprintf('Iteration: %i',iteration));
    for Mstate=1:M.state_no
        if ~find(M.C{Mstate}==mu(Mstate))
            error('Control %s is not available at state %i', M.CONTROLSTRING{mu(Mstate)}, Mstate);
        end
        %disp(sprintf('Control at state %i is %s', Mstate, M.CONTROLSTRING{mu(Mstate)}));
    end
    
    % Obtain Pmu
    Pmu=getPmu(M, mu);
    
    % Obtain gmu
    gmu=zeros(M.state_no,1);
    for Mstate=1:M.state_no
        gmu(Mstate)=M.g(Mstate, mu(Mstate));
    end
    
    % Obtain PmuLeftArrow and PmuRighArrow
    PmuLeft=sparse(M.state_no,M.state_no);
    PmuRight=sparse(M.state_no,M.state_no);
    for ii=1:M.state_no
        nind=find(Pmu(ii,:));
        for jj=nind
            % See if jj is in Spi
            if ~isempty(find(M.Spi==jj))
                PmuLeft(ii,jj)=Pmu(ii,jj);
            else
                PmuRight(ii,jj)=Pmu(ii,jj);
            end
        end
    end
    
    % Compute Jmu/hmu via J-h-v n*3 linear equations (new way)
    [Jmu, hmu] = getJhvACPC(Pmu, gmu, PmuRight);
    
    fprintf('Jmu(1)=%3.2f\n',Jmu(1));
    disp('======================');
    
    % find min_\mu Pmu Jmuk and set of controls that attain this min
    mu_up=zeros(1,M.state_no);
    U=cell(1,M.state_no);
    for ii=1:M.state_no
        pmuii=0;
        nind=find(M.P{mu(ii)}(ii,:));
        for jj=nind
            pmuii=pmuii+M.P{mu(ii)}(ii,jj)*Jmu(jj);
        end
        pmin=pmuii;
        umin=mu(ii);
        %disp(sprintf('State %i, control=%i', ii, mu(ii)));
        for uu=setdiff(M.C{ii},mu(ii))
            ptemp=0;
            nind=find(M.P{uu}(ii,:));
            for jj=nind
                ptemp=ptemp+M.P{uu}(ii,jj)*Jmu(jj);
            end
            %disp(sprintf('1st opt. pass: State %i, control old %s (cost %4.3f) control new %s (cost %4.3f)', ii, M.CONTROLSTRING{mu(ii)}, pmuii, M.CONTROLSTRING{uu}, ptemp));
            if pmin-ptemp>th
                umin=uu;
                pmin=ptemp;
            end
        end
        
        % Now find M(i) that retain the minimum from above
        Mii=[];
        for uu=M.C{ii}
            ptemp=0;
            nind=find(M.P{uu}(ii,:));
            for jj=nind
                ptemp=ptemp+M.P{uu}(ii,jj)*Jmu(jj);
            end
            %disp(sprintf('1st opt. pass: State %i, control min %s (cost %4.3f) control new %s (cost %4.3f)', ii, M.CONTROLSTRING{umin}, pmin, M.CONTROLSTRING{uu}, ptemp));
            if abs(ptemp-pmin)<th
                Mii=[Mii,uu];
            end
        end
        U{ii}=Mii;
        
        mu_up(ii)=umin;
        if umin~=mu(ii)
            %disp(sprintf('1st opt. pass: State %i, control updated from %s (cost %4.3f) to %s (cost %4.3f)', ii, M.CONTROLSTRING{mu(ii)}, pmuii, M.CONTROLSTRING{mu_up(ii)}, pmin));
        end
    end
    iteration=iteration+1;
    JJ=[JJ,Jmu];    
    if norm(mu_up-mu)~=0
        disp('1st opt. pass: Updating mu');
        mu=mu_up;
    else
        disp('1st opt. pass: Passed');
        mu_up=zeros(1,M.state_no);
        % find optimal for 2nd optimality equation
        for ii=1:M.state_no
            pmuii=M.g(ii,mu(ii));
            nind=find(M.P{mu(ii)}(ii,:));
            for jj=nind
                pmuii=pmuii+M.P{mu(ii)}(ii,jj)*hmu(jj);
                % See if jj is *not* in Spi
                if isempty(find(M.Spi==jj))
                    pmuii=pmuii+M.P{mu(ii)}(ii,jj)*Jmu(jj);
                end
            end
            pmin=pmuii;
            umin=mu(ii);
            
            for uu=setdiff(U{ii},mu(ii))
                ptemp=M.g(ii,uu);
                nind=find(M.P{uu}(ii,:));
                for jj=nind
                    ptemp=ptemp+M.P{uu}(ii,jj)*hmu(jj);
                    % See if jj is *not* in Spi
                    if isempty(find(M.Spi==jj))
                        ptemp=ptemp+M.P{uu}(ii,jj)*Jmu(jj);
                    end
                end
                %disp(sprintf('2nd opt. pass: State %i, control old %s (cost %4.3f) control new %s (cost %4.3f)', ii, M.CONTROLSTRING{mu(ii)}, pmuii, M.CONTROLSTRING{uu}, ptemp));
                if pmin-ptemp>th
                    umin=uu;
                    pmin=ptemp;
                end
            end
            
            mu_up(ii)=umin;
            if umin~=mu(ii)
                %disp(sprintf('2nd opt. pass: State %i, control updated from %s (cost %4.3f) to %s (cost %4.3f)', ii, M.CONTROLSTRING{mu(ii)}, pmuii, M.CONTROLSTRING{mu_up(ii)}, pmin));
            end
        end
        
        if norm(mu_up-mu)~=0
            disp('2nd opt. pass: Updating mu');
            mu=mu_up;
        else
            disp('Found optimal policy');
            break
        end
    end
    
end
%Check for optimality condition
for ii = 1:M.state_no
    %fprintf('Checking opt. condition for state %i\n', ii);
    pmuii=M.g(ii, mu(ii));
    nind=find(M.P{mu(ii)}(ii,:));
    for jj=nind
        pmuii=pmuii+M.P{mu(ii)}(ii,jj)*hmu(jj);
        % See if jj is *not* in Spi
        if isempty(find(M.Spi==jj))
            pmuii=pmuii+M.P{mu(ii)}(ii,jj)*Jmu(jj);
        end
    end
    pmin=pmuii;
    umin=mu(ii);
            
    for uu=setdiff(M.C{ii},mu(ii))
        ptemp=M.g(ii, uu);
        nind=find(M.P{uu}(ii,:));
        for jj=nind
            ptemp=ptemp+M.P{uu}(ii,jj)*hmu(jj);
            % See if jj is *not* in Spi
            if isempty(find(M.Spi==jj))
                ptemp=ptemp+M.P{uu}(ii,jj)*Jmu(jj);
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
disp('Optimality condition satisfied');