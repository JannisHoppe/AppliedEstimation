% function [S_bar] = predict(S,u,R)
% This function should perform the prediction step of MCL
% Inputs:
%           S(t-1)              4XM
%           v(t)                1X1
%           omega(t)            1X1
%           R                   3X3
%           delta_t             1X1
% Outputs:
%           S_bar(t)            4XM
function [S_bar] = predict(S,v,omega,R,delta_t)

%dynamic_noise_adaption = abs(v) > 10*eps && abs(omega) > 10*eps;
u_bar = [v*delta_t*cos(S(3,:));v*delta_t*sin(S(3,:));repmat(omega*delta_t,1,length(S(1,:)));zeros(1,length(S(1,:)))];
S_bar = S + u_bar + randn(length(S(:,1)),length(S(1,:))).*sqrt([R(1,1);R(2,2);R(3,3);0]);%*dynamic_noise_adaption;


end