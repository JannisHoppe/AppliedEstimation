function [ weights] = weight_particles( weights_in,post_seen)
%WEIGHT_PARTICLES Summary of this function goes here
%   Detailed explanation goes here

%particle weight without normalization
if post_seen == 0
weights_in = prod(weights_in,1);
weights= weights_in./sum(weights_in);
else
    
weights_help = prod(weights_in(1:end-1,:),1);
weights_help = weights_help./sum(weights_help);
weights_help = weights_help+weights_in(end,:)./sum(weights_in(end,:));
weights = weights_help./sum(weights_help);
end

