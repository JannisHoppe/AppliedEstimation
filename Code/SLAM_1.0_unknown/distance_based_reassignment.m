function [ feature_idx ] = distance_based_reassignment( particle,new_feature_pose,c,j )
%DISTANCE_BASED_REASSIGNMENT Summary of this function goes here
%   Detailed explanation goes here

feature_idx = c(j);
for counter = 1:1:length(c)
    features_coord(1,counter) = particle(6+(c(counter)-1)*7);
    features_coord(2,counter) = particle(7+(c(counter)-1)*7);
end

delta = features_coord-new_feature_pose;
dist = sqrt(delta(1,:).^2+delta(2,:).^2);
[min_dist,ind] = min(dist);

if min_dist < 60
    feature_idx = ind;
end

end

