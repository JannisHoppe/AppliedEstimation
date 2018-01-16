% this function realizes the update step of the Kalman filter for each
% feature
function part_bar = update_kalman(particle,z,c,Q_t,old_number_features,t_in,dist_reassign_on)
num_mes = size(c,2);
checked = zeros(1,old_number_features);
part_bar = particle;
help =0;
t=t_in;
number_double_updates =0;
N_count = 0;
for i = 1:num_mes
    feature_idx = c(i);
    p_idx = 6+(feature_idx-1)*7;
    double_update= 0;
    if feature_idx > old_number_features
        new_feature_pose = initialize_mean(z(:,i),particle(1:3));
        feature_idx_new = distance_based_reassignment(particle,new_feature_pose,c,i,old_number_features,dist_reassign_on);
        if feature_idx_new == feature_idx || old_number_features == 0
            feature_idx = feature_idx - number_double_updates;
            p_idx = 6+(feature_idx-1)*7;
            part_bar(p_idx:p_idx+1) = new_feature_pose;
            H = calculate_jacobian(particle(1:3),part_bar(p_idx:p_idx+1));
            H = special_mat_reshape(H);
            inv_H = H^-1;
            A = inv_H * Q_t * inv_H';
            part_bar(p_idx+2)= A(1);
            part_bar(p_idx+3) = A(2);
            part_bar(p_idx+4) = A(3);
            part_bar(p_idx+5) = A(4);
            part_bar(p_idx+6) = 1;
        else
            feature_idx = feature_idx_new;
            p_idx = 6+(feature_idx-1)*7;
            double_update = 1;
            number_double_updates = number_double_updates +1;
        end
        help= 1;
    end
        if (feature_idx <= old_number_features)
            checked(feature_idx) = 1;
            z_hat = observation_model_slam(particle(1:3),particle(p_idx:p_idx+1));
            H = calculate_jacobian(particle(1:3),particle(p_idx:p_idx+1));
            H = special_mat_reshape(H);
            Sigma = [particle(p_idx+2) particle(p_idx+3); 
                particle(p_idx+4) particle(p_idx+5)];
            Q = H * Sigma * H' + Q_t;
            inv_Q = Q^-1;
            K = Sigma * H' * inv_Q;
            part_bar(p_idx:p_idx+1) = particle(p_idx:p_idx+1) + K * (z(:,i)-z_hat);
            part_bar(p_idx+2:p_idx+5) = (eye(2)-K*H)*Sigma;
            part_bar(p_idx+6) = particle(p_idx+6) + 1*(1-double_update);
help =1;
        else
            if help ~=1
            fprintf('Soemthig went wrong\n');
            end
        end

end

for i=1:1:32
   if part_bar(12 + (i-1)*7) >= 0
    N_count = N_count+1;
   end   
end
part_bar(5) = N_count;
% reevaluate features that have not been assigned
discard = [];
for i=1:old_number_features
    if(checked(i)==0)
        p_idx = 6 + (i-1)*7;
        diff_range = sqrt((particle(1)-particle(p_idx)).^2+(particle(2)-particle(p_idx+1)).^2);
        if(diff_range < 500)
            part_bar(p_idx+6) = particle(p_idx+6) - 1;
            if(part_bar(p_idx+6)<0)
                discard = [discard p_idx];
                %part_bar = discard_feature(part_bar,p_idx);
            end
        end
    end
end

if (size(discard)>0)
    part_bar = discard_feature(part_bar,discard);
end
