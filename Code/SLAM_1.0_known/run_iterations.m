clear all;

number_iterations = 10;
fixed_post = 1;
VR_Resample = 1;
video_playback = 0;


for i =1:1:number_iterations
   results = run_SLAM_Stanford('control_Stanford.txt', 'MapStanford.txt',1,1,[264; 0; 1.57079632679490],2,video_playback,fixed_post,VR_Resample,i);
end

fixed_post = 0;

for i =1:1:number_iterations
   results = run_SLAM_Stanford('control_Stanford.txt', 'MapStanford.txt',1,1,[264; 0; 1.57079632679490],2,video_playback,fixed_post,VR_Resample,i);
end