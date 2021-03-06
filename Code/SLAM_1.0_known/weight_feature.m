% this function calculates the weight for a given feature based on the
% measurement likelihood.
function weight = weight_feature(z,z_hat,Q_inv,Q)

delta = z-z_hat;
delta(2,:) = mod(delta(2,:) + pi,2*pi)-pi;
exponent = -0.5*(delta(1,:).*(Q_inv(1,:).*delta(1,:) + Q_inv(3,:).*delta(2,:)) + delta(2,:).*(Q_inv(2,:).*delta(1,:) + Q_inv(4,:).*delta(2,:)));
determinant = (2*pi*(Q(1,:).*Q(4,:)-Q(3,:).*Q(2,:))).^-0.5;
weight = determinant.*exp(exponent);

end
