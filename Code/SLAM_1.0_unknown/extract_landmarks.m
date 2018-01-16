function [ landmark_data ] = extract_landmarks(S,true_coords)
%for each landmark extrakt the avaerage estimate. 

Number_features = S(5,1);
landmark_data = -1*ones(1,4*16);

for counter=1:1:Number_features
    x_est = S(6+(counter-1)*7);
    y_est = S(7+(counter-1)*7);
    sig11 = S(8+(counter-1)*7);
    sig22 = S(11+(counter-1)*7);
    
    for counter2=1:1:16
        dist(counter2) = sqrt((x_est-true_coords(counter2,1))^2+(y_est-true_coords(counter2,2))^2);
    end
    [min_dist,ind] = min(dist);
    
    landmark_data(1+(ind-1)*4) = x_est;
    landmark_data(2+(ind-1)*4) = y_est;
    landmark_data(3+(ind-1)*4) = sig11;
    landmark_data(4+(ind-1)*4) = sig22;
    
end


end

