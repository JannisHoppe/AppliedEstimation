% implementation of the measurement model for SLAM
% S : 3xM
% L : 2xM
% h : 2xM
function h = observation_model_slam(S,L)
    x = (L(1,:)-S(1,:));
    y = (L(2,:)-S(2,:));
    h1 = sqrt(x.^2 + y.^2);
    h2 = atan2(y,x) - S(3,:);
    h2 = mod(h2 + pi,2*pi)-pi;
    h = [h1;h2]; 
end