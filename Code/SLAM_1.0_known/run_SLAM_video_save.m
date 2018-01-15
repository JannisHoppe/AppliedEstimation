% runs the whole simulation for FAST SLAM 1.0 with known associations
function results =  run_SLAM_video_save(simoutfile, mapfile,show_estimate,show_gth,start_pose,verbose,video_playback,video_save,known_post,VR_resampling,run_number)
if nargin <7
    video_playback = 0; % Verbose = 0: no visual output, 1: estimates and groundtruth, 2: (1)+ covariance elipse
    known_post = 1;
    VR_resampling = 0;
end
help = 1;
%% Loading the map file
tic;
d = load(mapfile);

bound = [0 1424 0 1088];
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
    for counterrr=1:1:16
        handle_vec(counterrr) = plot(0,0,'c','erasemode','xor','LineWidth',5,'MarkerSize',5);
    end
end

W = d(:,2:3)';
Map_IDS = d(:,1)';
fid = fopen(simoutfile,'r');
if fid <= 0
    disp(sprintf('Failed to open simoutput file "%s"\n',simoutfile));
    return
end

%%
% Parameter Initialization
[S,R,Q,Lambda_psi,USE_KNOWN_ASSOCIATIONS,RESAMPLE_MODE,FIXED_POST_STATION,VR_RESAMPLE] = init(bound,start_pose,Map_IDS(end),known_post,VR_resampling);
%%
% Code initialization
% clc;
landmark_data = [];
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
    
    max_iterations = 150;
    
    %%plot saves
    plot_save_estimation_x = [];
    plot_save_estimation_y = [];
    
    plot_save_groundtruth_x = [];
    plot_save_groundtruth_y = [];
    
    plot_save_distance = [];
    plot_save_landmark_1 = [];
    plot_save_landmark_12 = [];
    
    %%read in video file
    obj = VideoReader('newVideo.mp4');
    endframe= obj.Duration*obj.FrameRate-1;
    load('time_vec.mat');
    
    result_video = strcat('results/robotVideo_',num2str(run_number));
    writerObj = VideoWriter(result_video,'Uncompressed AVI');
    writerObj.FrameRate = obj.FrameRate;
    
    open(writerObj);
    F(5400) = struct('cdata',[],'colormap',[]);
    
%     %VideoObejct for Graphs
%     graph_video = strcat('results/robotVideoGraphs_',num2str(run_number));
%     writerObjGraph = VideoWriter(graph_video,'Uncompressed AVI');
%     writerObjGraph.FrameRate = obj.FrameRate;
%     
%     open(writerObjGraph);
%     G(5400) = struct('cdata',[],'colormap',[]);
    %%
    % Main loop
    for q=1:endframe
        this_frame = readFrame(obj);
        time = obj.CurrentTime;
        if (abs(time-t_vec(count+2))<= 10*eps)
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
            [S,outliers] = SLAM(S,R,Q,z,known_associations,v,omega,Lambda_psi,Map_IDS,delta_t,count,USE_KNOWN_ASSOCIATIONS,RESAMPLE_MODE,FIXED_POST_STATION,VR_RESAMPLE);
            
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
            
            %Euclidean distance for robot pose
            distance = sqrt(rerr(1)^2+rerr(2)^2);
            plot_save_distance = [plot_save_distance distance];
            
            plot_save_estimation_x = [plot_save_estimation_x mu(1)];
            plot_save_estimation_y = [plot_save_estimation_y mu(2)];
            plot_save_groundtruth_x = [plot_save_groundtruth_x truepose(1)];
            plot_save_groundtruth_y = [plot_save_groundtruth_y truepose(2)];
            
%             % save landmarks
%             mu_landmark = [sum(S(4,:).*S(6,:));sum(S(4,:).*S(7,:))];
%             if mu_landmark(1) < 0
%                 plot_save_landmark_1 = [plot_save_landmark_1 NaN];
%             else
%                 landmark_coordinate_1 = landmarks(1,2:3)' - mu_landmark;
%                 landmark_dist_1 = sqrt(landmark_coordinate_1(1)^2 + landmark_coordinate_1(2)^2);
%                 plot_save_landmark_1 = [plot_save_landmark_1 landmark_dist_1];
%             end
%             
%             mu_landmark = [sum(S(4,:).*S(6+(12-1)*7,:));sum(S(4,:).*S(7+(12-1)*7,:))];
%             if mu_landmark(1) < 0
%                 plot_save_landmark_12 = [plot_save_landmark_12 NaN];
%             else
%                 landmark_coordinate_12 = landmarks(12,2:3)' - mu_landmark;
%                 landmark_dist_12 = sqrt(landmark_coordinate_12(1)^2 + landmark_coordinate_12(2)^2);
%                 plot_save_landmark_12 = [plot_save_landmark_12 landmark_dist_12];
%             end
            
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
                for counterrr=1:1:16
                    handle_vec(counterrr) = plot(0,0,'c','erasemode','xor','LineWidth',5,'MarkerSize',5);
                end
                for counter = 1:1:Map_IDS(end)
                    if S(5+(counter-1)*7,1)==1
                        mu_landmark = [sum(S(4,:).*S(6+(counter-1)*7,:));sum(S(4,:).*S(7+(counter-1)*7,:))];
                                    
                        sig11 = sum(S(4,:).*S(8+(counter-1)*7,:));
                        sig12 = sum(S(4,:).*S(9+(counter-1)*7,:));
                        sig21 = sum(S(4,:).*S(10+(counter-1)*7,:));
                        sig22 = sum(S(4,:).*S(11+(counter-1)*7,:));
                        sigma_landmark= [sig11, sig12;sig21,sig22];
                        pcov_landmark= abs(make_covariance_ellipses(mu_landmark,sigma_landmark));
                        set(handle_vec(counter),'xdata',pcov_landmark(1,:),'ydata',plot_transform(pcov_landmark(2,:)));
                    end
                end
            end
        end
        if n > 0 && show_gth&& verbose > 0
            %plot(truepose(1), truepose(2), 'gx');
            plot(plot_save_groundtruth_x,plot_transform(plot_save_groundtruth_y),'gx');
        end
        title(sprintf('Estimation at t = %d',count),'Interpreter','latex');
        set(gca,'FontName','Arial','FontSize',12);
        drawnow
        F(q) = getframe(gca);
        
        help = help +1;
        if  q == max_iterations
            break
        end
    end
%     fig2 = figure;
%     max1 = max(plot_save_distance);
%     max2 = max(plot_save_landmark_1);
%     if isnan(max2)
%         max2 = 10;
%     end
%     max3 = max(plot_save_landmark_12);
%     if isnan(max3)
%         max3 = 10;
%     end
%     for i=1:size(plot_save_distance)
%         clf(fig2);
%         subplot(3,1,1),plot(plot_save_distance(1:i)), title('Error Robot Pose'),xlim([0,15]),ylim([0,max1]);
%         subplot(3,1,2),plot(plot_save_landmark_1(1:i)), title('Error Pose Landmark 1'),xlim([0,15]),ylim([0,max2]);
%         subplot(3,1,3),plot(plot_save_landmark_12(1:i)), title('Error Pose Landmark 12'),xlim([0,15]),ylim([0,max3]);
%         G(i) = getframe(gcf);
%     end
    if video_save == 1
        for v = 1:endframe
            frame = F(v);
            writeVideo(writerObj,frame);
%             frame_graph = G(v);
%             writeVideo(writerObjGraph,frame_graph);
            if  v == max_iterations
                break
            end
        end
        close(writerObj);
        close(writerObjGraph);
    end
    
    
end

end
