% function S = systematic_resample(S_bar)
% This function performs systematic re-sampling
% Inputs:   
%           S_bar(t):       4XM
% Outputs:
%           S(t):           4XM
function S = systematic_resample(S_bar)

M = length(S_bar(1,:));
S = zeros(4,M);
CDF = cumsum(S_bar(4,:));

r = rand(1,1)/M;
for m = 1:1:M

i = find(CDF>=(r+(m-1)/M),1,'first');
S(:,m) = [ S_bar(1:3,i);1/M];
end


end