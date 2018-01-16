function [] = plot_Simulation_results(results,plot_err,plot_var,plot_feat_pos,plot_feat_var)
% plot

close all;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on')

if plot_err
    figure(1);
    clf;
    subplot(3,1,1);
    plot(results.err_x(:),'blue','LineWidth',1.5);
    title(sprintf('Estimation error on x'),'Interpreter','latex');%, mean error=%f, mean absolute err=%f', mean(results.err_x(:)),mean(abs(results.err_x(:)))),'Interpreter','latex');
    set(gca,'FontName','Arial','FontSize',12);
   % ylabel('pixles','Interpreter','latex');
    %xlabel('timestep','Interpreter','latex');
    subplot(3,1,2);
    plot(results.err_y(:),'blue','LineWidth',1.5);
    title(sprintf('Estimation error on y'),'Interpreter','latex'); %, mean error=%f, mean absolute err=%f', mean(results.err_y(:)),mean(abs(results.err_y(:)))),'Interpreter','latex');
    ylabel('pixles','Interpreter','latex');
    %xlabel('timestep','Interpreter','latex');
    set(gca,'FontName','Arial','FontSize',12);
    subplot(3,1,3);
    plot(results.err_t(:),'blue','LineWidth',1.5);
   % ylabel('pixles','Interpreter','latex');
    xlabel('timestep','Interpreter','latex');
    title(sprintf('Estimation error on theta'),'Interpreter','latex');%, mean error=%f, mean absolute err=%f',mean(results.err_t(:)),mean(abs(results.err_t(:)))),'Interpreter','latex');
    set(gca,'FontName','Arial','FontSize',12);
end

if plot_var
    figure(2);
    title('robot position variance over iterations in pixles','Interpreter','latex');
    clf;
    subplot(3,1,1);
    plot(results.sig_11(:),'blue','LineWidth',1.5);
        title('Estimation variance on x','Interpreter','latex')
    set(gca,'FontName','Arial','FontSize',12);
    subplot(3,1,2);
    plot(results.sig_22(:),'blue','LineWidth',1.5);
        title('Estimation variance on y','Interpreter','latex')
        ylabel('pixles','Interpreter','latex');
    set(gca,'FontName','Arial','FontSize',12);
    subplot(3,1,3);
    plot(results.sig_22(:),'blue','LineWidth',1.5);
    title('Estimation variance on theta','Interpreter','latex')
    xlabel('timestep','Interpreter','latex');
    set(gca,'FontName','Arial','FontSize',12);
end

    for i=1:1:16

        if plot_feat_pos
        figure('units','normalized','position',[0.15,0.3,0.4,0.5]); 
        subplot(2,1,1);
        plot(1:540,results.landmark_data(:,(i-1)*4+1),'blue','LineWidth',1.5);
        title(sprintf('Landmark %i: x coordinate estimate',i),'Interpreter','latex');
        ylabel('pixles','Interpreter','latex');
        xlabel('timestep','Interpreter','latex');
        hold on;
        plot(1:540,repmat(results.landmarks_true(i,1),540),'red','LineWidth',1.5);
        ylabel('pixles','Interpreter','latex');
        ylim([results.landmarks_true(i,1)-50,results.landmarks_true(i,1)+50]);
        legend('estimate','true x position');
        xlabel('timestep','Interpreter','latex');
        set(gca,'FontName','Arial','FontSize',12);
        
        subplot(2,1,2);
        plot(1:540,results.landmark_data(:,(i-1)*4+2),'blue','LineWidth',1.5);
        title(sprintf('Landmark %i: y coordinate estimate',i),'Interpreter','latex');
        xlabel('timestep','Interpreter','latex');
        ylabel('pixles','Interpreter','latex');
        hold on;
        plot(1:540,repmat(results.landmarks_true(i,2),540),'red','LineWidth',1.5);
        ylim([results.landmarks_true(i,2)-50,results.landmarks_true(i,2)+50]);
        xlabel('timestep','Interpreter','latex');
        ylabel('pixles','Interpreter','latex');
        legend('estimate','true y position');
        set(gca,'FontName','Arial','FontSize',12);
        end

        if plot_feat_var
        figure();
        subplot(2,1,1);
        plot(1:540,results.landmark_data(:,(i-1)*4+3),'blue','LineWidth',1.5);
        title(sprintf('landmark %i: distance estimate variance',i),'Interpreter','latex');
        xlabel('timestep','Interpreter','latex');
        ylabel('pixles','Interpreter','latex');
        set(gca,'FontName','Arial','FontSize',12);
        
        subplot(2,1,2);
        plot(1:540,results.landmark_data(:,(i-1)*4+4),'blue','LineWidth',1.5);
        title(sprintf('landmark %i: angle estimate',i),'Interpreter','latex');
        xlabel('timestep','Interpreter','latex');
        ylabel('pixles','Interpreter','latex');
        set(gca,'FontName','Arial','FontSize',12);
        end
        
    end

end

