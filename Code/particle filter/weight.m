% function S_bar = weight(S_bar,Psi,outlier)
%           S_bar(t)            4XM
%           outlier             1Xn
%           Psi(t)              1XnXM
% Outputs: 
%           S_bar(t)            4XM
function S_bar = weight(S_bar,Psi,outlier)
n = size(outlier,2);
M = size(S_bar,2);
for i = 1:n
    if (outlier(i))
        Psi(:,i,:) = ones(1,M);
    end
end
weights = prod(Psi,2);
weights = (1/sum(weights)).*weights;
S_bar(4,:) = weights;
end
