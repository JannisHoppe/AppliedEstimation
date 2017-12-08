function [ true_path ] = make_true_path(filepath)
%MAKE_TRUE_PATH Summary of this function goes here
%   Detailed explanation goes here
if nargin <1
    filepath = 'C:\Users\hoppe\Desktop\KTH\Applied Estimation\Project\Dataset\Textfiles\';
end
openpath = [filepath 'robot_path.txt'];
d = load([openpath]);

true_path(:,1:2) = d(:,2:3);
true_path(1,3) = pi/2;
for i = 2:1:length(true_path(:,1))
true_path(i,3) = atan2((true_path(i-1,2)-true_path(i,2)),(true_path(i,1)-true_path(i-1,1)));
true_path(i,3) = mod(true_path(i,3)+pi,2*pi)-pi;
end



end
