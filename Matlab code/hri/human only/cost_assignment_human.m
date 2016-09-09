function r=cost_assignment_human()

N_assemble=6;
N_fatigue=3;
N=N_assemble*N_fatigue;
keySet={'a0h','a1h','a2h','a3h','reset'};
valueSet={1,2,3,4,5};
%Action=containers.Map(keySet,valueSet);
state=states_human();
Na=length(valueSet);
r=zeros(N,Na);
r_assemble=zeros(N_assemble,Na);
r_fatigue=zeros(N_fatigue,Na);
%% assign assemble cost
r_assemble(6,5)=0.1;%reward finishing one assembly
%% assign fatigue cost

for i=1:4
    r_fatigue(1,i)=0.1;%reward working when fatigue level is low
    r_fatigue(2,i)=1;% neither reward or heavily penalize when fatigue level is medium
    r_fatigue(3,i)=3; %penalize working when fatigue level is high
end

for i=1:Na
    for j=1:N
        s=state(j,:)+1;%state starts from 0 but we need to start from 1
        r(j,i)=r_assemble(s(1),i)+r_fatigue(s(2),i);
    end
end


