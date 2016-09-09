function [x,A_opt]=MRP(M,dest,unreachablestates)
% Compute max reach. prob. and optimal policy using value iteration
x=zeros(1,M.state_no);
A_opt=zeros(1,M.state_no);
for state=dest
    x(state)=1;
    A_opt(state)=0;
end
for state=unreachablestates
    x(state)=0;
    A_opt(state)=-1;
end

reachablestates=setdiff(1:M.state_no,[unreachablestates,dest]);
itercount=0;

while true
    itercount=itercount+1;
    %x_prev=x;
    A_optprev=A_opt;
    for state=reachablestates
        Pmax=0;
        umax=-1;
        for uu=M.C{state}
            [a,indices,prob]=find(M.P{uu}(state,:));
            if sum(x(indices).*prob)>Pmax
                Pmax=sum(x(indices).*prob);
                umax=uu;
            end
        end
        if Pmax > x(state)
            x(state)=Pmax;
            A_opt(state)=umax;
        end
    end

    if A_optprev==A_opt
        fprintf('Value Iteration completed, iteration #=%i\n',itercount);
        break
    end
%     if x_prev==x
%         break
%     end
end