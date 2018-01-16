function [ feature_idx ] = distance_based_reassignment( particle,new_feature_pose,c,j,Number_old_features,activated )
% this function is used after the data association in the update process.
% The information is added, that any newly introduced feature cannot be
% closer to an existent feature than min_dist. If a feature is about to be
% introduced that violates this boundary, the data is matched to the
% closest feature and this feature is updated instead (maybe twice).

feature_idx = c(j);
if Number_old_features == 0
    return
end
for counter = 1:1:Number_old_features
    features_coord(1,counter) = particle(6+(counter-1)*7);
    features_coord(2,counter) = particle(7+(counter-1)*7);
end

delta = features_coord-new_feature_pose;
dist = sqrt(delta(1,:).^2+delta(2,:).^2);
[min_dist,ind] = min(dist);

if min_dist < 75 && activated 
    feature_idx = ind;
end

end

