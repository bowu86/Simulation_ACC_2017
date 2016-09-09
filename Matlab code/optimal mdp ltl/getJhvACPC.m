function [Jmu, hmu, vmu] = getJhvACPC(Pmu, gmu, PmuRight)
% This function computes the vector Jmu, hmu and vmu from Pmu and gmu for
% ACPC problems

if size(Pmu,1) ~= size(Pmu,2) && size(Pmu,1) ~= length(gmu)
    error('Size of Pmu and gmu wrong');
end

state_no = length(gmu);

X = zeros(3*state_no);

I = eye(state_no);

% first row
X(1:state_no, 1:state_no) = I - Pmu;
% second row
X((state_no+1):(2*state_no), 1:state_no) = I - PmuRight;
X((state_no+1):(2*state_no), (state_no+1):(2*state_no)) = I - Pmu;
% third row
X((2*state_no+1):(3*state_no), (state_no+1):(2*state_no)) = I - PmuRight;
X((2*state_no+1):(3*state_no), (2*state_no+1):(3*state_no)) = I - Pmu;

b = zeros(3*state_no, 1);
b((state_no+1):(2*state_no)) = gmu;

% y = X\b;
% Since X is not full rank, using psudo-inv
y = pinv(X) * b;
%y = pseudoinverse(X) * b;

% y = lsqr(sparse(X),sparse(b));
%y = lsqr(sparse(X),sparse(b), 1e-06, 1000000);

tol = 1e-5;
if norm(X*y-b) > tol
    warning('Norm error %e greater than tol %e', norm(X*y-b), tol);
end

Jmu = y(1:state_no); hmu = y((state_no+1):(2*state_no)); vmu = y((2*state_no+1):(3*state_no));