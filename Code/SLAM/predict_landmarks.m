%map_ids -- id's of the landmarks that were seen this time step
% z = measurement for this time step
function [S_bar] = predict_landmarks(S,Q_t,z,map_ids,num_landmarks)
    S_bar = S;
    weights_landmarks = [];
    for j= 1:num_landmarks
        
        %Feature measured ? 
        if ismember(j,map_ids)
            %Feature seen before?
            m_idx = 5+(j-1)*7;
            if(max(S(m_idx,:))==0)%no
                S_bar(m_idx,:) = 1; %now it was seen
                S_bar(m_idx+1:m_idx+2,:) = initialize_mean(z(:,find(map_ids==j)),[S(1,:);S(2,:);S(3,:)]);
                H = calculate_jacobian(S(1:3,:),S_bar(m_idx+1:m_idx+2,:));
                inv_H = special_mat_inverse(H);
                S_bar(m_idx+3,:) = Q_t(1)*(inv_H(1,:)).^2 + Q_t(4) * (inv_H(2,:)).^2 ;
                S_bar(m_idx+4,:) = Q_t(1)* (inv_H(1,:).*inv_H(3,:)) + Q_t(4) *(inv_H(2,:).*inv_H(4,:));
                S_bar(m_idx+5,:) = Q_t(1)* (inv_H(1,:).*inv_H(3,:)) + Q_t(4) *(inv_H(2,:).*inv_H(4,:));
                S_bar(m_idx+6,:) = Q_t(1) * (inv_H(3,:)).^2 + Q_t(4)*(inv_H(4,:)).^2;
                weights_landmarks = [weights_landmarks; ones(1,size(S,2))];
            else
                z_hat = observation_model_slam(S(1:3,:),S(m_idx+1:m_idx+2,:));
                H = calculate_jacobian(S(1:3,:),S(m_idx+1:m_idx+2,:));
                Q = [Q_t(1)+H(1,:).*(H(1,:).*S(m_idx+3,:) + H(2,:).*S(m_idx+5,:)) + H(2,:).*(H(1,:).*S(m_idx+4,:) + H(2,:).*S(m_idx+6,:)); 
                    H(3,:).*(H(1,:).*S(m_idx+3,:)+ H(2,:).*S(m_idx+5,:)) + H(4,:).*(H(1,:).*S(m_idx+4,:)+ H(2,:).*S(m_idx+6,:));
                    H(1,:).*(H(3,:).*S(m_idx+3,:)+ H(4,:).*S(m_idx+5,:)) + H(2,:).*(H(3,:).*S(m_idx+4,:)+ H(4,:).*S(m_idx+6,:));
                    Q_t(4) + H(3,:).*(H(3,:).*S(m_idx+3,:)+ H(4,:).*S(m_idx+5,:)) + H(4,:).*(H(3,:).*S(m_idx+4,:)+ H(4,:).*S(m_idx+6,:))];
                inv_Q = special_mat_inverse(Q);
                K = [inv_Q(1,:).*(H(1,:).*S(m_idx+3,:) + H(2,:).*S(m_idx+4,:)) + inv_Q(3,:).*(H(3,:).* S(m_idx+3,:) + H(4,:).*S(m_idx+4,:));
                    inv_Q(2,:).*(H(1,:).*S(m_idx+3,:) + H(2,:).*S(m_idx+4,:)) + inv_Q(4,:).*(H(3,:).* S(m_idx+3,:) + H(4,:).*S(m_idx+4,:));
                    inv_Q(1,:).*(H(1,:).*S(m_idx+5,:) + H(2,:).*S(m_idx+6,:)) + inv_Q(3,:).*(H(3,:).* S(m_idx+5,:) + H(4,:).*S(m_idx+6,:));
                    inv_Q(2,:).*(H(1,:).*S(m_idx+5,:) + H(2,:).*S(m_idx+6,:)) + inv_Q(4,:).*(H(3,:).* S(m_idx+5,:) + H(4,:).*S(m_idx+6,:))];
                % update mean for landmark
                S_bar(m_idx+1,:) = S(m_idx+1,:) + K(1,:).* (z(1) - z_hat(1,:)) + K(2,:).*(z(2)-z_hat(2,:));
                S_bar(m_idx+2,:) = S(m_idx+2,:) + K(3,:).*(z(1) - z_hat(1,:)) + K(4,:).*(z(2)-z_hat(2,:));
                % update covariance for landmark
                S_bar(m_idx+3,:) = -S(m_idx+3,:).*(H(1,:).*K(1,:) + H(3,:).*K(2,:) - 1) - S(m_idx+5,:).*(H(2,:).*K(1,:) + H(4,:).*K(2,:)) ;
                S_bar(m_idx+4,:) = -S(m_idx+4,:).*(H(1,:).*K(1,:) + H(3,:).*K(2,:) - 1) - S(m_idx+6,:).*(H(2,:).*K(1,:) + H(4,:).*K(2,:));
                S_bar(m_idx+5,:) = -S(m_idx+3,:).*(H(1,:).*K(3,:) + H(3,:).*K(4,:)) - S(m_idx+5,:).*(H(2,:).*K(3,:) + H(4,:).*K(4,:)-1);
                S_bar(m_idx+6,:) = -S(m_idx+4,:).*(H(1,:).*K(3,:) + H(3,:).*K(4,:)) - S(m_idx+6,:).*(H(2,:).*K(3,:) + H(4,:).*K(4,:)-1);
                weights_landmarks = [weights_landmarks; weight_feature(z(:,find(map_ids==j)),z_hat,Q)];
              

            end
%         else
%             S_bar(5*j:5*j+4,:) = S(5*j:5*j+4,:);
        end
    end
    S_bar(4,:) = weight_particles(weights_landmarks);
end

