% function S_bar = weight(S_bar,Psi,outlier)
%           S_bar(t)            4XM
%           outlier             1Xn
%           Psi(t)              1XnXM
% Outputs: 
%           S_bar(t)            4XM
function weight = weight_feature(z,z_hat,Q_inv)

delta = z-z_hat;
delta(2,:) = mod(delta(2,:) + pi,2*pi)-pi;
exponent = -0.5*(delta(1,:).*(Q_inv(1).*delta(1,:) + Q_inv(3).*delta(2,:)) + delta(2,:).*(Q_inv(2).*delta(1,:) + Q_inv(4).*delta(2,:)));
determinant = (2*pi*(Q_inv(1).*Q_inv(4)-Q_inv(3).*Q_inv(2))).^-0.5;
weight = determinant.*exp(exponent);

end
