function [ N,c,weight ] = associate_and_weight( weights,z,N_before,S_bar_part )
% This function calculates the weights of the particles and is responsible
% for data association. By choosing the MLE estimate for data association
% only one of the calculated weights for each landmark remains per
% particle. The product of these weight is taken as non normalized particle
% weight. A new feature is introduced, if the measurement likelihood for
% all features is smaller than a random guess.

if isempty(weights)
    N = size(z,2);
    weight = 1;
    for counter = 1:1:N
        c(1,counter) = counter;
    end
else
num_measurements = length(weights(1,:)); 
c = zeros(1,num_measurements); %association vector
number_features_before = length(weights(:,1)); % number of features seen in the last time step. 
counter_new_feature = -1*ones(1,num_measurements); % in order to adjust the associations vector in the end
weight = 1; % weight of this particle for resampling. 
for counter = 1:1:num_measurements
    weight_help = [];
    if max(weights(:,counter)) > 1e-7
    weight_help = weights(:,counter)./sum(weights(:,counter)); %normalize, to see if one landmark is described significantly better then the others (relatively!!!!)% normieren k�nnte falsch sein
    else
    weight_help = weights(:,counter);
    end
    weight_help(end+1) = 1/N_before + 0.05; % reference weight for getting a new feature
    [~, c(1,counter)] = max(weight_help);
    if c(1,counter) > number_features_before
        counter_new_feature(1,counter) = max(counter_new_feature(1,:))+1;
    end   
end
counter_new_feature(counter_new_feature==-1) = 0;

c(1,:) = c(1,:)+ counter_new_feature;
N = max(number_features_before,max(c(1,:))); %vielleicht falsch

for counter = 1:1:num_measurements
    if c(1,counter) <= N_before % multiply the weight of all features seen before
        weight = weight*weights(c(1,counter),counter);    
        
    end
end
% if weight==1
%     weight = 0; % if the weigh
% end
end  
end

