A=[.5,.2,.3; .2,.3,.5; 0.1,.2,.7]
B=A; B(:,1)=0; eig(B)
C=(eye(3)-B)

for ii=1:10
    d=rand(3,1)*1000;
    c=rand(3,1)*1000;
    for jj=1:3
        if c(jj)>d(jj)
            temp1=c(jj);
            c(jj)=d(jj);
            d(jj)=temp1;
        end
    end
    c-d
    A*c-A*d
    pause
end

