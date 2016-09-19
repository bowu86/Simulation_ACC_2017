function r=cost_assignment()

Na=10;
N_assemble=6;
N_robot=2;
N_fatigue=3;
N_trust=3;
N=N_assemble*N_robot*N_fatigue*N_trust;
r=zeros(N,Na);
r_assemble=zeros(N_assemble,Na);
r_robot=zeros(N_robot,Na);
r_fatigue=zeros(N_fatigue,Na);
r_trust=zeros(N_trust,Na);
keySet={'a0r','a1r','a2r','a3r','a0h','a1h','a2h','a3h','reset','repair'};
valueSet={1,2,3,4,5,6,7,8,9,10};
Action=containers.Map(keySet,valueSet);
state=states();
%% assign assemble cost

r_assemble(1,Action('a0r'))=1;%cost on the road
r_assemble(1,Action('a1r'))=1;
r_assemble(1,Action('a0h'))=1;
r_assemble(1,Action('a1h'))=1;

r_assemble(2,Action('a1r'))=1;%cost on the road
r_assemble(2,Action('a1h'))=1;

r_assemble(3,Action('a0r'))=1;
r_assemble(3,Action('a0h'))=1;

r_assemble(4,Action('a2r'))=1;%cost on the road
r_assemble(4,Action('a2h'))=1;

r_assemble(5,Action('a3r'))=1;%cost on the road
r_assemble(5,Action('a3h'))=1;

r_assemble(6,Action('reset'))=0.1;%reward finishing one assembly
%% assign robot cost
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r_robot(1,Action('a0r'))=1;%energy cost
r_robot(1,Action('a1r'))=1;
r_robot(1,Action('a2r'))=1;
r_robot(1,Action('a3r'))=1;
r_robot(2,Action('repair'))=2;%cost of repair
%% assign fatigue cost
for i=1:4
r_fatigue(1,i)=2;%penalize idle while there is no fatigue
r_fatigue(2,i)=1;%
r_fatigue(3,i)=0.1; %reward idle when fatigue level is high
end

for i=[5:8,10]
    r_fatigue(1,i)=0.1;%reward working when fatigue level is low
    r_fatigue(2,i)=1;% neither reward or heavily penalize when fatigue level is medium
    r_fatigue(3,i)=3; %penalize working when fatigue level is high
end
%% assign trust cost
for i=1:4
r_trust(1,i)=3;%penalize low trust 
r_trust(2,i)=1;
r_trust(3,i)=0.1; %reward high trust
end

for i=1:N_trust
r_trust(i,10)=5;%penalize lost of trust due to failure
end

for i=1:Na
    for j=1:N
        s=state(j,:)+1;%state starts from 0 but we need to start from 1
        r(j,i)=r_assemble(s(1),i)+r_robot(s(2),i)+r_fatigue(s(3),i)+r_trust(s(4),i);
    end
end


