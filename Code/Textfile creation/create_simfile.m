sensor_max_distance = 500; %max sensor distance in pixles
sigma_Q = [25,15*2*pi/360];
disturb_measurements = 1;

true_path = make_true_path();
load('C:\Users\hoppe\Desktop\KTH\Applied Estimation\test\Code\Textfile creation\time_vec.mat');
t_vec = make_time_vec(time_vec);
input = make_input(true_path,t_vec);

map = load('C:\Users\hoppe\Desktop\KTH\Applied Estimation\test\Dataset\Textfiles\MapStanford.txt');

[number_landmarks,measurements] = make_measurements(map,true_path,sensor_max_distance,sigma_Q,disturb_measurements);

%outfile = [t_vec, input, true_path, measurements];
