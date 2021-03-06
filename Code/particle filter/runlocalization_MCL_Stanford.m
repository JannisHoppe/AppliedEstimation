% function runlocalization_track(simoutfile, mapfile,show_estimate,show_gth,show_odo,verbose)
% This function is the entrance point to the code. 
function runlocalization_MCL_Stanford(simoutfile, mapfile,show_estimate,show_gth,start_pose,verbose,video_playback)
if nargin <7
    video_playback = 0; % Verbose = 0: no visual output, 1: estimates and groundtruth, 2: (1)+ covariance elipse
end
help = 1;
%% Loading the map file
tic;
d = load(mapfile);

bound = [0 1424 0 1088];
%%
% Parameter Initialization
[S,R,Q,Lambda_psi] = init(bound,start_pose);
%%
% Code initialization
% clc;

sensorpose = [0;0;0];
if verbose
    fige = figure(1); % Estimated Movement and Map
    clf(fige);
    
    margin = 0;
    xmin = 0 - margin;
    xmax = 1424 + margin;
    ymin = 0 - margin;
    ymax = 1088 + margin;
   
    figure(fige);
    drawLandmarkMap(mapfile);
    hold on;
    title('Estimated Map and Movement');
    parts_handle = plot(0,0,'b.','erasemode','xor');
end
landmarks = load(mapfile);
hcovs = [];
if verbose > 1
    figure(fige);
    hcovs = plot(0,0,'r','erasemode','xor','LineWidth',5,'MarkerSize',5);   
end

W = d(:,2:3)';
Map_IDS = d(:,1)';
fid = fopen(simoutfile,'r');
if fid <= 0
  disp(sprintf('Failed to open simoutput file "%s"\n',simoutfile));
  return
end
flines = {};
while 1
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    flines = {flines{:} line};
end
fclose(fid);

h = [];
he = [];
hg = [];
errpose = [];
count = 0;
gth = [];
total_outliers = 0;
t = 0;
sigma = cov(S(1,:),S(2,:));
var_theta = var(S(3,:));
s_sigma = zeros(3,3);
s_sigma(1:2,1:2) = sigma;
s_sigma(3,3) = var_theta;
sigma_save = s_sigma(:);

if video_playback == 1
%%plot saves
plot_save_estimation_x = [];
plot_save_estimation_y = [];

plot_save_groundtruth_x = [];
plot_save_groundtruth_y = [];

%%read in video file
obj = VideoReader('newVideo.mp4');
endframe= obj.Duration*obj.FrameRate-1;
load('time_vec.mat');

%%
% Main loop
for v=1:endframe
    this_frame = readFrame(obj);
    time = obj.CurrentTime;
    if (abs(time-t_vec(count+1))<= 10*eps)
        count = count + 1;
        if count > length(flines)
            break;
        end
        line = flines{count};
        values = sscanf(line, '%f');
        pt = t;
        t = values(1);
        delta_t = t - pt;
        v = values(2);
        omega = values(3);
        truepose = values(4:6);
        gth = [gth truepose];
        n = values(7);
        if (n > 0) 
            bearings = values(9:3:end);
            ranges = values(10:3:end);
            ids = values(8:3:end);
        else
            bearings = [];
            ranges = [];
            ids = [];
        end
        z = [ranges';bearings'];
        known_associations = ids';
        [S,outliers] = mcl(S,R,Q,z,known_associations,v,omega,W,Lambda_psi,Map_IDS,delta_t,count);

        total_outliers = total_outliers + outliers;
        mu = mean(S(1:3,:),2);
        sigma = cov(S(1,:),S(2,:));
        rerr = truepose - mu;
        var_theta = var(S(3,:));
        s_sigma = zeros(3,3);
        s_sigma(1:2,1:2) = sigma;
        s_sigma(3,3) = var_theta;
        sigma_save = [sigma_save s_sigma(:)];
        rerr(3) = mod(rerr(3)+pi,2*pi)-pi;
        errpose = [errpose rerr];
        for k = 1:length(h)
            delete(h(k))
        end
        plot_save_estimation_x = [plot_save_estimation_x mu(1)];
        plot_save_estimation_y = [plot_save_estimation_y mu(2)];
        plot_save_groundtruth_x = [plot_save_groundtruth_x truepose(1)];
        plot_save_groundtruth_y = [plot_save_groundtruth_y truepose(2)];

    end
    clf(fige)
    thisax = axes('Parent', fige);
    image(this_frame, 'Parent', thisax);
    hold on;
    plot(landmarks(:,2), plot_transform(landmarks(:,3)), 'ko');
    
    if n > 0 && show_estimate && verbose > 0
        if verbose > 1
            %plot(mu(1), mu(2), 'rx')
            plot(plot_save_estimation_x,plot_transform(plot_save_estimation_y),'rx');
        end
        parts_handle = plot(0,0,'b.','erasemode','xor');
        set(parts_handle,'xdata',S(1,:),'ydata',plot_transform(S(2,:)));
        if verbose > 1
            figure(fige);
            hcovs = plot(0,0,'r','erasemode','xor','LineWidth',5,'MarkerSize',5);
            help_mu = [mu(1);plot_transform(mu(2));mu(3)];
            pcov= make_covariance_ellipses(help_mu,sigma);
            set(hcovs,'xdata',pcov(1,:),'ydata',pcov(2,:));
        end  
    end        
    if n > 0 && show_gth&& verbose > 0
        %plot(truepose(1), truepose(2), 'gx');
        plot(plot_save_groundtruth_x,plot_transform(plot_save_groundtruth_y),'gx');
    end  
    title(sprintf('t= %d, total outliers=%d, current outliers=%d',time,total_outliers,outliers));
    
    drawnow
    help = help +1;
    
end

else
    
% Main loop
while 1
    count = count + 1;
    if count > length(flines)
        break;
    end
    line = flines{count};
    values = sscanf(line, '%f');
    pt = t;
    t = values(1);
    delta_t = t - pt;
    v = values(2);
    omega = values(3);
    truepose = values(4:6);
    gth = [gth truepose];
    n = values(7);
    if (n > 0) 
        bearings = values(9:3:end);
        ranges = values(10:3:end);
        ids = values(8:3:end);
    else
        bearings = [];
        ranges = [];
        ids = [];
    end
    z = [ranges';bearings'];
    known_associations = ids';
    [S,outliers] = mcl(S,R,Q,z,known_associations,v,omega,W,Lambda_psi,Map_IDS,delta_t,count);
        
    total_outliers = total_outliers + outliers;
    mu = mean(S(1:3,:),2);
    sigma = cov(S(1,:),S(2,:));
    rerr = truepose - mu;
    var_theta = var(S(3,:));
    s_sigma = zeros(3,3);
    s_sigma(1:2,1:2) = sigma;
    s_sigma(3,3) = var_theta;
    sigma_save = [sigma_save s_sigma(:)];
    rerr(3) = mod(rerr(3)+pi,2*pi)-pi;
    errpose = [errpose rerr];
    for k = 1:length(h)
        delete(h(k))
    end
    h = [];
   
    if n > 0 && show_estimate && verbose > 0
        if verbose > 1
            plot(mu(1), mu(2), 'rx')
        end
        RE = [cos(mu(3)) -sin(mu(3)); 
              sin(mu(3)) cos(mu(3))];

        xsE = mu(1:3) + [RE * sensorpose(1:2); sensorpose(3)];
        set(parts_handle,'xdata',S(1,:),'ydata',S(2,:));
        he = [];  
        if verbose > 1
            pcov= abs(make_covariance_ellipses(mu,sigma));
            set(hcovs,'xdata',pcov(1,:),'ydata',pcov(2,:));
        end
        title(sprintf('t= %d, total outliers=%d, current outliers=%d',count,total_outliers,outliers));
        axis([xmin xmax ymin ymax]) 
    end        
    if n > 0 && show_gth&& verbose > 0
        plot(truepose(1), truepose(2), 'gx');
        RG = [cos(truepose(3)) -sin(truepose(3)); 
              sin(truepose(3)) cos(truepose(3))];
       
        xsG = truepose(1:3) + [RG * sensorpose(1:2); sensorpose(3)];

        hg = [];  
        if verbose > 2        
            for k = 1:n
                    h2 = plot(xsG(1)+[0 ranges(k)*cos(xsG(3)+bearings(k))], ...
                            xsG(2)+[0 ranges(k)*sin(xsG(3)+bearings(k))], 'g');

                    hg = [hg h2];
            end
        end
        axis([xmin xmax ymin ymax]) 
    end  
    h = [he hg];
    
    drawnow
    help = help +1;
    varianz_theta(count) = var(S(3,:));
end   
    
end
time = toc;
maex = mean(abs(errpose(1,:)));
mex = mean(errpose(1,:));
maey = mean(abs(errpose(2,:)));
mey = mean(errpose(2,:));
maet = mean(abs(errpose(3,:)));
met = mean(errpose(3,:));
display(sprintf('mean error(x, y, theta)=(%f, %f, %f)\nmean absolute error=(%f, %f, %f)\ntotal_time =%f',mex,mey,met, maex,maey,maet,time));
if verbose >1
    figure(2);
    clf;
    subplot(3,1,1);
    plot(errpose(1,:));
    title(sprintf('error on x, mean error=%f, mean absolute err=%f',mex,maex));
    subplot(3,1,2);
    plot(errpose(2,:));
    title(sprintf('error on y, mean error=%f, mean absolute err=%f',mey,maey));
    subplot(3,1,3);
    plot(errpose(3,:));
    title(sprintf('error on theta, mean error=%f, mean absolute err=%f',met,maet));
    
    figure(3);
    clf;
    subplot(3,1,1);
    plot(sigma_save(1,:));
    title('\Sigma(1,1)');
    subplot(3,1,2);
    plot(sigma_save(5,:));
    title('\Sigma(2,2)');
    subplot(3,1,3);
    plot(sigma_save(9,:));
    title('\Sigma(3,3)');
end
