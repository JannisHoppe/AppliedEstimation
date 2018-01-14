%initialize simulation 
function [S,R,Q,Lambda_psi,USE_KNOWN_ASSOCIATIONS,RESAMPLE_MODE,FIXED_POST_STATION,VR_RESAMPLING] = init(bound,start_pose,number_landmarks,known_post,VR_resampling)

USE_KNOWN_ASSOCIATIONS = 0;
RESAMPLE_MODE = 2; %0=no resampling, 1=Multinomial resampling, 2=Systematic Resampling
FIXED_POST_STATION = known_post;
VR_RESAMPLING = VR_resampling;


M = 1000;
if ~isempty(start_pose)
    S_x = [repmat(start_pose,1,M); 1/M*ones(1,M)];
else
    S_x = [rand(1,M)*(bound(2) - bound(1)) + bound(1);
         rand(1,M)*(bound(4) - bound(3)) + bound(3);
         rand(1,M)*2*pi-pi;
         1/M*ones(1,M)];
end

if USE_KNOWN_ASSOCIATIONS == 1
Landmark_init = [0;-1;-1;-1;-1;-1;-1]; % seenflag;mu_x;mu_y;Sig11;Sig12;Sig21;Sig22
S_M = repmat(Landmark_init,number_landmarks,M);
S = [S_x;S_M];
else
S_x = [S_x; zeros(1,M)]; %Number seen Landmarks
Landmark_init = [-1;-1;-1;-1;-1;-1;-1]; % mu_x;mu_y;Sig11;Sig12;Sig21;Sig22, counter
S_M = repmat(Landmark_init,number_landmarks*2,M);
S = [S_x;S_M];   
end
R = diag([4^2 4^2 (5*2*pi/360)^2]); %process noise covariance matrix
Q = diag([150;(5*2*pi/360)^2]); % measurement noise covariance matrix
Lambda_psi = 0.0000001;
end
