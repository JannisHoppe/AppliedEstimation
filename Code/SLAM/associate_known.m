% function [outlier,Psi] = associate_known(S_bar,z,W,Lambda_psi,Q,known_associations)
%           S_bar(t)            4XM
%           z(t)                2Xn
%           W                   2XN
%           Lambda_psi          1X1
%           Q                   2X2
%           known_associations  1Xn
% Outputs: 
%           outlier             1Xn
%           Psi(t)              1XnXM
function [outlier,Psi] = associate_known(S_bar,z,W,Lambda_psi,Q,known_associations)
% FILL IN HERE

%BE SURE THAT YOUR innovation 'nu' has its second component in [-pi, pi]

% also notice that you have to do something here even if you do not have to maximize the likelihood.

M = length(S_bar(1,:));
n = length(z(1,:));

z_j = zeros(2,M);
nu_ij = zeros(2,M);

outlier = zeros(1,n);
Psi = zeros(1,n,M);

for i = 1:1:n 
    association = known_associations(1,i); 
    z_j= observation_model(S_bar,W,association);
    nu_ij = z(:,i)-z_j;
    nu_ij(2,:)= mod(nu_ij(2,:)+pi,2*pi)-pi;    
    psi(:) = 1/((2*pi)*(det(Q)^0.5))*exp(-0.5*(nu_ij(1,:).^2/Q(1)+ nu_ij(2,:).^2*Q(4)^-1));       
    outlier(1,i) = (1/M*sum(psi) <= Lambda_psi);
    Psi(1,i,:) = psi(:);
end

end
