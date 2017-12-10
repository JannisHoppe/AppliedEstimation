function [ mu ] = initialize_mean(z,x)

theta_help = z(2)+x(3,:);
theta_help = mod(theta_help+pi,2*pi)-pi;
mu_x = x(1,:)+z(1)*cos(theta_help(1,:));
mu_y = x(2,:)+z(1)*sin(theta_help(1,:));

mu = [mu_x;mu_y];


end

