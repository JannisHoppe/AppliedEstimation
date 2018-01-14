% This function performs one iteration of the FAST SLAM 1.0 Algorithm
% with known associations.

function [S,outliers] = SLAM(S,R,Q,z,known_associations,v,omega,Lambda_psi,Map_IDS,delta_t,t,USE_KNOWN_ASSOCIATIONS,RESAMPLE_MODE,FIXED_POST_STATION,VR_RESAMPLE)

M = size(S,2);
old_weights = S(4,:);
S_bar_state = S(1:4,:);
[S_bar_state] = predict_state(S_bar_state,v,omega,R,delta_t,VR_RESAMPLE);
S_bar = [S_bar_state;S(5:end,:)];

if USE_KNOWN_ASSOCIATIONS
    map_ids = zeros(1,size(z,2));
    for i = 1 : size(z,2)
        map_ids(i) = find(Map_IDS == known_associations(i));
    end
    [S_bar] = predict_landmarks(S_bar,Q,z,map_ids,length(Map_IDS(1,:)),FIXED_POST_STATION);
else
    % other folder
end

outliers = 0;
if outliers
    display(sprintf('warning, %d measurements were labeled as outliers, t=%d',sum(outlier), t));
end

if VR_RESAMPLE == 1 && abs(v)<10*eps && abs(omega)<10*eps
    calculated_weights = S_bar(4,:);
    weights = old_weights.*calculated_weights;
    weights = weights./sum(weights);
    S = [S_bar(1:3,:);weights;S_bar(5:end,:)];
else
switch RESAMPLE_MODE
    case 0
        S = S_bar;
    case 1
        if old_weights ~= 1/M*ones(1,M)
             calculated_weights = S_bar(4,:);
             weights = old_weights.*calculated_weights;
             weights = weights./sum(weights);
             S_bar = [S_bar(1:3,:);weights;S_bar(5:end,:)];   
        end
        S = multinomial_resample(S_bar);
    case 2
        if old_weights ~= 1/M*ones(1,M)
             calculated_weights = S_bar(4,:);
             weights = old_weights.*calculated_weights;
             weights = weights./sum(weights);
             S_bar = [S_bar(1:3,:);weights;S_bar(5:end,:)];   
        end
        S = systematic_resample(S_bar);
end
end
end
