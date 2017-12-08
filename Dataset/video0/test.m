clear all
% 
%     clear all; close all; clc;
%     v = VideoReader('video.mov');
%     cutVideo = VideoWriter('newVideo');
%     cutVideo.FrameRate = v.FrameRate;
% 
%     open(cutVideo);
% 
%     %Read and write each frame.
%     for k= (190*30):(370*30)
%         vidFrames = read(v,k);
%         writeVideo(cutVideo,vidFrames)
%         k
%     end
% 
%     close(cutVideo);


%%%%%%%%%%%%%%%%%%

thisfig = figure();
obj = VideoReader('newVideo.mp4');
endframe= obj.Duration*obj.FrameRate-1;

for k = 1 : endframe  %fill in the appropriate number
this_frame = readFrame(obj);
thisax = axes('Parent', thisfig);
image(this_frame, 'Parent', thisax);
time = obj.CurrentTime;
title(thisax, sprintf('Frame #%d and Time %d', k, time));
time_vec(k,1) = time; 
hold on;
%plot([0:1000],[0:1000]);
pause(1/obj.Framerate);
if  mod(k,10) == 0
    hi = 1;
end
clf(thisfig)
end
