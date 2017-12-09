%map_ids -- id's of the landmarks that were seen this time step
function [S_bar] = predict_landmarks(S,Q,map_ids,num_landmarks)
    S_bar = S;
    for j= 1:num_landmarks
        %Feature measured ? 
        if ismember(j,map_ids)
            %Feature seen before?
            m_idx = 5+(j-1)*6;
            if(max(S(m_idx,:))==0)%no
                S_bar(m_idx,:) = 1; %now it was seen
                %mean : S_bar(5*j+1:5*j+2,:)
                H = calculate_jacobian(S_bar,m_idx);
            else
                
            end
%         else
%             S_bar(5*j:5*j+4,:) = S(5*j:5*j+4,:);
        end
    end
    
end