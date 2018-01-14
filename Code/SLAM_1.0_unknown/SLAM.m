% function [S,outliers] = mcl(S,R,Q,z,known_associations,u,M,Lambda_psi,Map_IDS,delta_t,t)
% This function should perform one iteration of the FAST SLAM 1.0 algorithm
% without known associations.
function [S,current_weights] = SLAM(S,R,Q,z,known_associations,v,omega,Lambda_psi,Map_IDS,delta_t,t,USE_KNOWN_ASSOCIATIONS,RESAMPLE_MODE,FIXED_POST_STATION,VR_RESAMPLE)

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
    weight_particles = zeros(1,length(S_bar(1,:)));
    for particle =1:1:length(S_bar(1,:))   
        N_before = S_bar(5,particle);
        weights = measurement_likelihoods(S_bar(:,particle),z,Q);
        [N,c,weight] = associate_and_weight(weights,z,N_before,S_bar(:,particle));
        weight_particles(1,particle) = weight;
        S_bar(:,particle) = update_kalman(S_bar(:,particle),z,c,Q,N_before,t);       
    end
    S_bar(4,:) = weight_particles./sum(weight_particles);
end

current_weights = S_bar(4,:);

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
