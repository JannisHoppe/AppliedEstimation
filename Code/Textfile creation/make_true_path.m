function [ true_path ] = make_true_path()
%MAKE_TRUE_PATH Summary of this function goes here
%   Detailed explanation goes here
d = load('robot_path.txt');

true_path(:,1:2) = d(:,2:3);
true_path(:,2) = 1088- true_path(:,2);
true_path(1,3) = pi/2;
for i = 1:1:length(true_path(:,1))-1
true_path(i,3) = atan2((true_path(i+1,2)-true_path(i,2)),(true_path(i+1,1)-true_path(i,1)));
true_path(i,3) = mod(true_path(i,3)+pi,2*pi)-pi;
end
true_path(i+1,3) = true_path(i,3);



end
