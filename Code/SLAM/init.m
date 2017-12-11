% function [S,R,Q,Lambda_psi] = init(bound,start_pose)
% This function initializes the parameters of the filter.
% Outputs:
%			S(0):			4XM
%			R:				3X3
%			Q:				2X2
%           Lambda_psi:     1X1
%           start_pose:     3X1
function [S,R,Q,Lambda_psi,USE_KNOWN_ASSOCIATIONS,RESAMPLE_MODE,FIXED_POST_STATION] = init(bound,start_pose,number_landmarks)
M = 1000;
if ~isempty(start_pose)
    S_x = [repmat(start_pose,1,M); 1/M*ones(1,M)];
else
    S_x = [rand(1,M)*(bound(2) - bound(1)) + bound(1);
         rand(1,M)*(bound(4) - bound(3)) + bound(3);
         rand(1,M)*2*pi-pi;
         1/M*ones(1,M)];
end

Landmark_init = [0;-1;-1;-1;-1;-1;-1]; % seenflag;mu_x;mu_y;Sig11;Sig12;Sig21;Sig22
S_M = repmat(Landmark_init,number_landmarks,M);
S = [S_x;S_M];

R = diag([10 10 1*2*pi/360]); %process noise covariance matrix
Q = diag([250;20*2*pi/360]); % measurement noise covariance matrix
Lambda_psi = 0.0000001;
USE_KNOWN_ASSOCIATIONS = 1;
RESAMPLE_MODE = 2; %0=no resampling, 1=Multinomial resampling, 2=Systematic Resampling
FIXED_POST_STATION = 1;
end
