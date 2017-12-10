function [ mu ] = initialize_mean(z,x)

theta_help = z(2)+x(3,:);
mu_x = z(1)*cos(theta_help(1,:));
mu_y = z(1)*sin(theta_help(1,:));

mu = [mu_x;mu_y],


end

