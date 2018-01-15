% prediction of the current robot pose according to the motion model. When
% no motion is done in the current time step and the Variance reduction
% resampling is tunred on no noise is added.
function [S_bar] = predict_state(S,v,omega,R,delta_t,VR_RESAMPLE)

M = length(S(1,:));
u_bar = [v*delta_t*cos(S(3,:));v*delta_t*sin(S(3,:));repmat(omega*delta_t,1,length(S(1,:)));zeros(1,length(S(1,:)))];
if VR_RESAMPLE == 1
S_bar = S + u_bar+ randn(length(S(:,1)),length(S(1,:))).*sqrt([R(1,1);R(2,2);R(3,3);0])*(abs(v)>10*eps && abs(omega)>10*eps);
else
S_bar = S + u_bar+ randn(length(S(:,1)),length(S(1,:))).*sqrt([R(1,1);R(2,2);R(3,3);0]);
end
end