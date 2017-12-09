% function S_bar = weight(S_bar,Psi,outlier)
%           S_bar(t)            4XM
%           outlier             1Xn
%           Psi(t)              1XnXM
% Outputs: 
%           S_bar(t)            4XM
function S_bar = weight(z,z_hat,Q)

delta = z-z_hat;
exponent = -0.5*(delta(1,:).*(Q(1,:).*delta(1,:) + Q(3,:).*delta(2,:)) + delta(2,:).*(Q(2,:).*delta(1,:) + Q(4,:).*delta(2,:)));
determinant = (2*pi*(Q(1,:).*Q(4,:)-Q(3,:).*Q(2,:))).^-0.5;
fact = transpose(determinant.*exp(exponent));

% kommt sp�ter
% for i = 1:1:n
%     if outlier(1,i) == 1
%         fact(i,:) = 1;
%     else
%         fact(i,:) = Psi(1,i,:);                  
%     end
% end

p = prod(fact,1);
%normalization
p = p/sum(p);

end