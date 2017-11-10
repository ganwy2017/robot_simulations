clear all;
close all;

global x_
global y_
global z_
global plot_size

% ---- Visualization ----

error = [];
x_ = 4;
y_ = 4;
z_ = 4;

plot_size = 400;

% ---- Sim time ----

dt = 0.01;
run = true;
runtime = 0;
t_end = 2000;

% ---- Robot params ----
mass = 0.1;
Ixx = 8.7952e-3;
Iyy = 5.14714e-3;
Izz = 1.3624726e-2;
u_noise = [3 0.01 0.01 0];
robot = quad_model(mass, Ixx, Iyy, Izz, u_noise);

% ---- Desired State ----
yaw = 0;
goals(:,:,1) = [0,5,5; 0,0,0];
goals(:,:,2) = [5,5,5; 0,0,0];  
goals(:,:,3) = [5,-5,5; 0,0,0];
goals(:,:,4) = [-5,-5,5; 0,0,0];

des = [0,0,5; 0,0,0];

fprintf("Simulate for %.02f seconds\n",t_end*dt);
while(run)
    if length(error) > plot_size
        error(1) = [];
    end
    robot = robot.PID_pos_control(des,yaw,dt);
    error(end+1) = norm(des(1,:)-robot.x(1:3));
    
    if error(end) < 0.1
        if isempty(goals)
            run = false;
        else
            des = goals(:,:,1);
            goals(:,:,1) = [];
        end
    end
    
    
    pts = robot.transform();
    T = robot.thrust_vector();
    visualization(pts, T, des, error);
    
    pause(0.01)
    runtime = runtime + 1;
end

fprintf("Simulation end\n");