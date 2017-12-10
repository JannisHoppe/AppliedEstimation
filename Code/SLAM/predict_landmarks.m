%map_ids -- id's of the landmarks that were seen this time step
function [S_bar] = predict_landmarks(S,Q_t,map_ids,num_landmarks)
    S_bar = S;
    for j= 1:num_landmarks
        weight_landmarks = [];
        %Feature measured ? 
        if ismember(j,map_ids)
            %Feature seen before?
            m_idx = 5+(j-1)*6;
            if(max(S(m_idx,:))==0)%no
                S_bar(m_idx,:) = 1; %now it was seen
                %mean : S_bar(m_idx+1:m_idx+2,:)
                H = calculate_jacobian(S(1:3,:),S_bar(m_idx+1:m_idx+2,:));
                inv_H = special_mat_inverse(H);
                S_bar(m_idx+3,:) = Q_t(1)*(inv_H(1,:)).^2 + Q_t(4) * (inv_H(2,:)).^2 ;
                S_bar(m_idx+4,:) = Q_t(1)* (inv_H(1,:).*inv_H(3,:)) + Q_t(4) *(inv_H(2,:).*inv_H(4,:));
                S_bar(m_idx+5,:) = Q_t(1) * (inv_H(3,:)).^2 + Q_t(4)*(inv_H(4,:)).^2;
                weights_landmarks = [weights_landmarks; ones(1,size(S,2))];
            else
                z_hat = observation_model_slam(S(1:3,:),S(m_idx+1:m_idx+2,:));
                H = calculate_jacobian(S(1:3,:),S(m_idx+1:m_idx+2,:));
                Q = 0;
                
            end
%         else
%             S_bar(5*j:5*j+4,:) = S(5*j:5*j+4,:);
        end
    end
    
end

