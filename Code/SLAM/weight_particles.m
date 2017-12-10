function [ weights ] = weight_particles( weights )
%WEIGHT_PARTICLES Summary of this function goes here
%   Detailed explanation goes here

%particle weight without normalization
weights = prod(weights,1);
weights = weights./sum(weights);

end

