function [Past,err]=getPast(P)
% if ~checkStochastic(P)
%     error('P not stochastic!');
% end
Psum=zeros(size(P,1), size(P,1));
Past=zeros(size(P,1), size(P,1));

Nmax=10000;
th=0.0001;
for ii=0:Nmax-1
    Psum=Psum+P^ii;
    Pprev=Past;
    Past=Psum./(ii+1);
    err=norm(Past-Pprev,1);
    if err<th
        %disp(sprintf('passing threshold, iteration=%i',ii));
        break
    end    
end