function Transition=parse_transition_ltl_robot()
N=108;
Na=6;
keySet={'a0r','a1r','a2r','a3r','reset','repair'};
valueSet={1,2,3,4,5,6};
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

