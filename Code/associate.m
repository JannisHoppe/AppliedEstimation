% function [outlier,Psi] = associate(S_bar,z,W,Lambda_psi,Q)
%           S_bar(t)            4XM
%           z(t)                2Xn
%           W                   2XN
%           Lambda_psi          1X1
%           Q                   2X2
% Outputs: 
%           outlier             1Xn
%           Psi(t)              1XnXM
function [outlier,Psi] = associate(S_bar,z,W,Lambda_psi,Q)

M = length(S_bar(1,:));
n = length(z(1,:));
N = length(W(1,:));

z_j = zeros(2,M);
nu_ij = zeros(2,M);
psi_ij = zeros(M,N);

outlier = zeros(1,n);
Psi = zeros(1,n,M);

for i = 1:1:n
    for j = 1:1:N    
    z_j= observation_model(S_bar,W,j);
    nu_ij = z(:,i)-z_j;
    nu_ij(2,:)= mod(nu_ij(2,:)+pi,2*pi)-pi;
    % the next line is not implemented very nicely because this only works
    % for uncorrelated measurements. Since this assumption is valid for all
    % problems in this lab I decided to do it like this anyways.
    psi_ij(:,j) = 1/((2*pi)*(det(Q)^0.5))*exp(-0.5*(nu_ij(1,:).^2/Q(1)+ nu_ij(2,:).^2*Q(4)^-1));    
    end
    [psi,c] = max(psi_ij,[],2);   
    outlier(1,i) = (1/M*sum(psi) <= Lambda_psi);
    Psi(1,i,:) = psi(:);
end


end
