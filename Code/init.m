% function [S,R,Q,Lambda_psi] = init(bound,start_pose)
% This function initializes the parameters of the filter.
% Outputs:
%			S(0):			4XM
%			R:				3X3
%			Q:				2X2
%           Lambda_psi:     1X1
%           start_pose:     3X1
function [S,R,Q,Lambda_psi] = init(bound,start_pose)
M = 1000;
if ~isempty(start_pose)
    S = [repmat(start_pose,1,M); 1/M*ones(1,M)];
else
    S = [rand(1,M)*(bound(2) - bound(1)) + bound(1);
         rand(1,M)*(bound(4) - bound(3)) + bound(3);
         rand(1,M)*2*pi-pi;
         1/M*ones(1,M)];
end
   % plot(S(1,:),S(2,:),'rx')
% Below here you may want to experiment with the values but these seem to work for most datasets.

R = diag([4 4 10*2*pi/360]); %process noise covariance matrix
Q = diag([15;0.15]); % measurement noise covariance matrix
Lambda_psi = 0;

end
