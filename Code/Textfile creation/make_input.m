function [ input ] = make_input(true_path,t_vec)
%MAKE_INPUT Summary of this function goes here
%   Detailed explanation goes here

for i = 1:1:length(true_path(:,1))-1
input(i,1) = sqrt((true_path(i+1,1)-true_path(i,1))^2+(true_path(i+1,2)-true_path(i,2))^2)/(t_vec(i+1)-t_vec(i));
input(i,2) = (true_path(i+1,3)-true_path(i,3))/(t_vec(i+1)-t_vec(i));
end

end

