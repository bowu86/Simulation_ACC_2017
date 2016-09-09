function stochastic=checkStochastic(Pmu)

tol = 1e-8;

if size(Pmu,1)~=size(Pmu,2)
    disp('Pmu is not a square matrix!');
    stochastic=false;
    return
end

for ii=1:size(Pmu,1)
    if abs(sum(Pmu(ii,:)) - 1) > tol
        rowsum = sum(Pmu(ii,:));
        disp(sprintf('Pmu row %i does not sum to %2.3f instead of 1, Pmu not stochastic', full(rowsum), ii));
        stochastic=false;
        return
    end    
end
stochastic=true;
