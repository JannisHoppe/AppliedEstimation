% function h = observation_model(S,W,j)
% This function is the implementation of the observation model
% The bearing should lie in the interval [-pi,pi)
% Inputs:
%           S           4XM
%           W           2XN
%           j           1X1
% Outputs:  
%           h           2XM
function h = observation_model(S,W,j)

angle = mod(atan2(repmat(W(2),1,length(S(1,:)))-S(2,:),repmat(W(1),1,length(S(1,:)))-S(1,:))-S(3,:)+repmat(pi,1,length(S(1,:))),repmat(2*pi,1,length(S(1,:))))-repmat(pi,1,length(S(1,:)));
dist = sqrt((repmat(W(1),1,length(S(1,:)))-S(1,:)).^2 + (repmat(W(2),1,length(S(1,:)))-S(2,:)).^2);

h = [dist;angle];

end