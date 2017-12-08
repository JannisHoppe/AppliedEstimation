function [number_landmarkss, measurements ] = make_measurements( map,true_path,sensor_max_distance,sigma_Q,disturb_measurements)
%MAKE_MEASUREMENTS Summary of this function goes here
%   Detailed explanation goes here

measurements = cell(540,3);
number_landmarks = map(end,1);
number_landmarkss = zeros(540,1) ;

for i = 1:1:length(true_path(:,1))
    for j = 1:1:number_landmarks
        
        dist(j) = sqrt((true_path(i,1)-map(j,2))^2+(true_path(i,2)-map(j,3))^2) + ((2*sigma_Q(1))*rand(1)- sigma_Q(1))*disturb_measurements;
        if dist(j) > sensor_max_distance
            flag(j) = 0;
        else
            flag(j) = j;
        end
        theta(j) = atan2(map(j,3)-true_path(i,2),map(j,2)-true_path(i,1))-true_path(i,3);
        theta(j) = mod(theta(j)+pi,2*pi)-pi+((2*sigma_Q(2))*rand(1)- sigma_Q(2))*disturb_measurements;            
   %     theta_(j) = theta(j)*360/(2*piadsasda
    end
    
    for j =1:1:number_landmarks
        if flag(j) ~= 0
        number_landmarkss(i) = number_landmarkss(i)+1;
        measurements{i,1} = [measurements{i,1} theta(j)];
        measurements{i,2} = [measurements{i,2} dist(j)];
        measurements{i,3} = [measurements{i,3} flag(j)];
        end
    end
end

