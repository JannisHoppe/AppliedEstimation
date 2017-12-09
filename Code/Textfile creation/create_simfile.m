sensor_max_distance = 500; %max sensor distance in pixles
sigma_Q = [25,5*2*pi/360];
disturb_measurements = 1;

true_path = make_true_path();
load('time_vec_vid.mat');
t_vec = make_time_vec(time_vec);
input = make_input(true_path,t_vec);

map = load('MapStanford.txt');

[number_landmarks,measurements] = make_measurements(map,true_path,sensor_max_distance,sigma_Q,disturb_measurements);

fid = fopen( 'control_Stanford.txt', 'wt' );

for i=1:1:length(true_path(:,1))
    fprintf( fid, '%f %f %f %f %f %f %f ', t_vec(i),input(i,1), input(i,2), true_path(i,1), true_path(i,2), true_path(i,3), number_landmarks(i));
    for j = 1:1:number_landmarks(i)
        bearings = measurements{i,1};
        ranges = measurements{i,2};
        ids = measurements{i,3};
        fprintf(fid, '%f %f %f ', ids(j), bearings(j), ranges(j));
    end
    fprintf(fid, '\n');
end
fclose(fid);