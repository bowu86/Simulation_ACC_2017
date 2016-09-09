function Transition=parse_transition_ltl_human()
N=18;

keySet={'a0h','a1h','a2h','a3h','reset'};
valueSet={1,2,3,4,5};
Na=length(valueSet);
Action=containers.Map(keySet,valueSet);
eps=0.001;
fileID = fopen('transition.txt');

C = textscan(fileID,'%d %d %d %f %s');
fclose(fileID);

from=C{1};
to=C{3};
p=C{4};
a=C{5};
Transition=zeros(N,N,Na);
L=length(from);
for i = 1:L
    act=Action(a{i});
    f=from(i)+1;
    t=to(i)+1;
    Transition(f,t,act)=p(i);
end

