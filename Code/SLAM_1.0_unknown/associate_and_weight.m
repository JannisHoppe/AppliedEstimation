function [ N,c,weight ] = associate_and_weight( weights )
%ASSOCIATE Summary of this function goes here
%   Detailed explanation goes here
num_measurements = length(weights(1,:)); 
c = zeros(1,num_measurements); %association vector
number_features_before = length(weights(:,1)); % number of features seen in the last time step. 
counter_new_feature = -1*ones(1,num_measurements); % in order to adjust the associations vector in the end
weight = 1; % weight of this particle for resampling. 
for counter = 1:1:num_measurements
    weight_help = [];
    weight_help = weights(:,num_measurements)./sum(weights(:,num_measurements)); %normalize, to see if one landmark is described significantly better then the others (relatively!!!!)% normieren k�nnte falsch sein
    weight_help(end+1) = 0.5; % reference weight for getting a new feature
    [~, c(1,counter)] = max(weight_help);
    if c(1,counter) > number_features_before
        counter_new_feature(1,counter) = max(counter_new_feature(1,:))+1;
    end   
end
counter_new_feature(counter_new_feature==-1) = 0;

c(1,:) = c(1,:)+ counter_new_feature;
N = max(number_features_before,max(c(1,:)));

for counter = 1:1:num_measurements
    if c(1,counter) <= num_features_before % multiply the weight of all features seen before
        weight = weight*weights(c(1,counter),counter);       
    end
end
% if weight==1
%     weight = 0; % if the weigh
% end
        
end
