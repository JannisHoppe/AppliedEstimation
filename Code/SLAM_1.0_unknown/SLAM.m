% function [S,outliers] = mcl(S,R,Q,z,known_associations,u,M,Lambda_psi,Map_IDS,delta_t,t)
% This function should perform one iteration of Monte Carlo Localization
% Inputs:
%           S(t-1)              4XM
%           R                   3X3
%           Q                   2X2
%           z                   2Xn
%           known_associations  1Xn
%           u                   3X1
%           W                   2XN
%           t                   1X1
%           delta_t             1X1
%           Lambda_psi          1X1
%           Map_IDS             1XN
% Outputs:
%           S(t)                4XM
%           outliers            1X1
function [S,outliers] = SLAM(S,R,Q,z,known_associations,v,omega,Lambda_psi,Map_IDS,delta_t,t,USE_KNOWN_ASSOCIATIONS,RESAMPLE_MODE,FIXED_POST_STATION)

S_bar_state = S(1:4,:);
[S_bar_state] = predict_state(S_bar_state,v,omega,R,delta_t);
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
        S_bar(5,particle) = N;
        S_bar(:,particle) = update_kalman(S_bar(:,particle),z,c,Q,N_before);       
    end
    S_bar(4,:) = weight_particles./sum(weight_particles);
end

outliers = 0;
if outliers
    display(sprintf('warning, %d measurements were labeled as outliers, t=%d',sum(outlier), t));
end

switch RESAMPLE_MODE
    case 0
        S = S_bar;
    case 1
        S = multinomial_resample(S_bar);
    case 2
        S = systematic_resample(S_bar);
end
end
