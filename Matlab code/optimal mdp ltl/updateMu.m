% Display and check control
disp('======================');
disp(sprintf('Iteration: %i',iteration));
for ii=1:P.state_no
    if ~find(P.C{ii}==mu(ii))
        error('Control %s is not available at state %i', CONTROLSTRING{mu(ii)}, ii);
    end
    disp(sprintf('Control at state %i is %s', ii, CONTROLSTRING{mu(ii)}));
end

% Obtain Pmu
Pmu=getPmu(P, mu);

% Obtain gmu
gmu=zeros(P.state_no,1);
for ii=1:P.state_no
    gmu(ii)=CONTROLCOST(mu(ii));
end

% Obtain PmuLeftArrow and PmuRighArrow
PmuLeft=sparse(P.state_no,P.state_no);
PmuRight=sparse(P.state_no,P.state_no);
for ii=1:P.state_no
    nind=find(Pmu(ii,:));
    for jj=nind
        % See if jj is in Spi
        if ~isempty(find(P.Spi==jj))
            PmuLeft(ii,jj)=Pmu(ii,jj);
        else
            PmuRight(ii,jj)=Pmu(ii,jj);
        end
    end
end

% % Obtain Pmutilde and gmutilde
% if find(eig(eye(P.state_no)-PmuRight)==0)
%     error('I-PmuRight not invertible, probabily due to improper policy');
% end
% 
% Pmutilde=inv(eye(P.state_no)-PmuRight)*PmuLeft;
% gmutilde=inv(eye(P.state_no)-PmuRight)*gmu;
% 
% % % Compute Jmu/hmu via P* (old way)
% [Pmutildeast,err]=getPast(Pmutilde);
% if err>0.001
%     error('error too large, %2.3f',err);
% end
% 
% Hmutilde=inv(eye(P.state_no)-Pmutilde+Pmutildeast)-Pmutildeast;
% % 
% % Jmu=Pmutildeast*gmutilde;
% % hmu=Hmutilde*gmutilde;

% Compute Jmu/hmu via J-h-v n*3 linear equations (new way)
[Jmu, hmu] = getJhvACPC(Pmu, gmu, PmuRight);

% % check for correctness of Jmu and hmu
% disp(sprintf('norm(Pmutilde*Jmu-Pmu*Jmu)=%2.1f',norm(Pmutilde*Jmu-Pmu*Jmu)));
% disp(sprintf('norm((gmutilde+Pmutilde*hmu)-(gmu+Pmu*hmu+PmuRight*Jmu))=%2.1f',norm((gmutilde+Pmutilde*hmu)-(gmu+Pmu*hmu+PmuRight*Jmu))));

disp(['Jmu=',mat2str(Jmu,3)]);
disp('======================');

% find min_\mu Pmu Jmuk and set of controls that attain this min
mu_up=zeros(1,P.state_no);
U=cell(1,P.state_no);
for ii=1:P.state_no
    pmuii=0;
    nind=find(P.P{mu(ii)}(ii,:));
    for jj=nind
        pmuii=pmuii+P.P{mu(ii)}(ii,jj)*Jmu(jj);
    end
    pmin=pmuii;
    umin=mu(ii);
    %disp(sprintf('State %i, control=%i', ii, mu(ii)));
    for uu=setdiff(P.C{ii},mu(ii))
        ptemp=0;
        nind=find(P.P{uu}(ii,:));
        for jj=nind
            ptemp=ptemp+P.P{uu}(ii,jj)*Jmu(jj);
        end
        %disp(sprintf('1st opt. pass: State %i, control old %s (cost %4.3f) control new %s (cost %4.3f)', ii, CONTROLSTRING{mu(ii)}, pmuii, CONTROLSTRING{uu}, ptemp));
        if pmin-ptemp>th
            umin=uu;
            pmin=ptemp;
        end
    end
    
    % Now find M(i) that retain the minimum from above
    Mii=[];
    for uu=P.C{ii}
        ptemp=0;
        nind=find(P.P{uu}(ii,:));
        for jj=nind
            ptemp=ptemp+P.P{uu}(ii,jj)*Jmu(jj);
        end
        %disp(sprintf('1st opt. pass: State %i, control min %s (cost %4.3f) control new %s (cost %4.3f)', ii, CONTROLSTRING{umin}, pmin, CONTROLSTRING{uu}, ptemp));
        if abs(ptemp-pmin)<th
            Mii=[Mii,uu];
        end
    end
    U{ii}=Mii;
    
    mu_up(ii)=umin;
    if umin~=mu(ii)
        disp(sprintf('1st opt. pass: State %i, control updated from %s (cost %4.3f) to %s (cost %4.3f)', ii, CONTROLSTRING{mu(ii)}, pmuii, CONTROLSTRING{mu_up(ii)}, pmin));
    end
end

if norm(mu_up-mu)~=0
    disp('1st opt. pass: Updating mu');
    mu=mu_up;
else
    disp('1st opt. pass: Passed');
    mu_up=zeros(1,P.state_no);
    % find optimal for 2nd optimality equation
    for ii=1:P.state_no
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
        
        for uu=setdiff(U{ii},mu(ii))
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
                umin=uu;
                pmin=ptemp;
            end
        end
        
        mu_up(ii)=umin;
        if umin~=mu(ii)
            disp(sprintf('2nd opt. pass: State %i, control updated from %s (cost %4.3f) to %s (cost %4.3f)', ii, CONTROLSTRING{mu(ii)}, pmuii, CONTROLSTRING{mu_up(ii)}, pmin));
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

% fprintf('norm of ACPC optimality cond. error: %2.2e\n', norm((Jmu + hmu) - (gmu + Pmu*hmu + PmuRight*Jmu)));