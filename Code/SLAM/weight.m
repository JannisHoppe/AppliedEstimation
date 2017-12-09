% function S_bar = weight(S_bar,Psi,outlier)
%           S_bar(t)            4XM
%           outlier             1Xn
%           Psi(t)              1XnXM
% Outputs: 
%           S_bar(t)            4XM
function S_bar = weight(S_bar,Psi,outlier)
% FILL IN HERE

%BE CAREFUL TO NORMALIZE THE FINAL WEIGHTS

n = length(outlier(1,:));
M = length(S_bar(1,:));

fact = ones(n,M);

for i = 1:1:n
    if outlier(1,i) == 1
        fact(i,:) = 1;
    else
        fact(i,:) = Psi(1,i,:);                  
    end

p = prod(fact,1);

%normalization
p = p/sum(p);

S_bar(4,:) = p;

end
