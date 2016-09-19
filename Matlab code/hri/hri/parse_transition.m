function Transition=parse_transition()
N=108;
Na=10;
keySet={'a0r','a1r','a2r','a3r','a0h','a1h','a2h','a3h','reset','repair'};
valueSet={1,2,3,4,5,6,7,8,9,10};
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

%% make sure that each row of the transition matrix sums to 1
for i=1:Na
    for j=1:N
        s=sum(Transition(j,:,i));
        if abs(s-1)>eps
            Transition(j,j,i)=1;
            
        end
    end
end