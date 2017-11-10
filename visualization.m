function [] = visualization( model_pts, T, des, data )
%VISUALIZATION Summary of this function goes here
%   Detailed explanation goes here
    global x_
    global y_
    global z_
    global plot_size
    persistent first
    persistent h_
    limit = 200;
    
    if isempty(first)
        h1 = figure;
        set(h1, 'Position',[300 400 1100 500]);
        first(1) = 0;
        h_ = [];
    end
    
    if size(h_) >= [limit, 3]
        h_(1,:) = [];
    end
    h_(end+1,:) = T(2,:);
    
    subplot(1,2,1);
    hold off;
    plot3(model_pts(:,1),model_pts(:,2),model_pts(:,3));
    hold on;
    plot3(T(:,1),T(:,2),T(:,3));
    hold on;
    plot3(h_(:,1),h_(:,2),h_(:,3));
    hold on;
    scatter3(des(1,1),des(1,2),des(1,3),15);
    grid on
    title('Simulation');
    xlim([T(2,1)-x_ T(2,1)+x_]);
    ylim([T(2,2)-y_ T(2,2)+y_]);
    zlim([T(2,3)-z_ T(2,3)+z_]);
    
    subplot(1,2,2);
    hold off;
    plot(data);
    title('Tracking Error');
    xlim([0 plot_size+50]);
    ylim([data(end)-5 data(end)+5]);
    grid on
end

