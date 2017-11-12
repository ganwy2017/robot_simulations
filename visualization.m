function [] = visualization( model_pts, T, des, data, time )
    global x_
    global y_
    global z_
    global dt
    global plot_size
    persistent first
    persistent h_

    t = dt*(max(-1,time-length(data))+1:1:time);
    
    if isempty(first)
        h1 = figure;
        set(h1, 'Position',[300 400 1100 500]);
        first(1) = 0;
        h_ = [];
    end    
    if size(h_) >= [plot_size, 3]
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
    
    pt = max(z_,T(2,3));
    xlim([T(2,1)-x_ T(2,1)+x_]);
    ylim([T(2,2)-y_ T(2,2)+y_]);
    zlim([pt-z_ pt+z_]);
    
    win_z = 3;
    pt = max(win_z,data(end));
    
    subplot(1,2,2);
    hold off;
    plot(t,data);
    title('Tracking Error');
    xlim([t(1) t(end)+dt+t(end)*0.1]);
    ylim([pt-win_z, pt+win_z]);
    grid on
end

