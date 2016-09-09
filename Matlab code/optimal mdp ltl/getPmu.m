function Pmu=getPmu(P, mu)
Pmu=sparse(P.state_no, P.state_no);
for uu=1:P.control_no
    prob=P.P{uu};
    for ii=1:P.state_no
        if uu==mu(ii)            
            nind=find(prob(ii,:));
            for jj=nind
                Pmu(ii,jj)=prob(ii,jj);
            end
        end
    end
end

% Check if Pmu is stochastic
if ~checkStochastic(Pmu)
    error('Pmu not stochastic');
end
