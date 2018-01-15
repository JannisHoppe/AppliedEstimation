function [ landmark_data ] = extract_landmarks( S )
%for each landmark extrakt the avaerage estimate. 

maximum = size(S,1);
idx_start = 5;
k = 7;
counter = 0;
landmark_data = [];

while idx_start+counter*k <= maximum
weights = S(4,:);
mean_pos_x = sum(weights.*S(idx_start+counter*k+1,:));
mean_pos_y = sum(weights.*S(idx_start+counter*k+2,:));
mean_sig11 = sum(weights.*S(idx_start+counter*k+3,:));
mean_sig22 = sum(weights.*S(idx_start+counter*k+6,:));
landmark_data =  [landmark_data,mean_pos_x,mean_pos_y,mean_sig11,mean_sig22];
counter = counter+1;
end


end

