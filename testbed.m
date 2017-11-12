clear all;
close all;

global x_
global y_
global z_
global dt
global plot_size

% ---- Visualization ----

error = [];
x_ = 2;
y_ = 2;
z_ = 2;

plot_size = 100;

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
u_noise = [3 0.001 0.001 0.001];
robot = quad_model(mass, Ixx, Iyy, Izz, u_noise);

% ---- Desired State ----
traj = trajectory(0.1);
traj.des = [0,0,5,0,0,0;
            0,5,5,0,0,0;
            5,5,5,0,0,0;
            5,0,5,0,0,0;
            0,0,5,0,0,0;
            0,0,0,0,0,0];
yaw = 0;

fprintf("Simulate for %.02f seconds and %i waypoints\n",t_end*dt, size(traj.des,1));
while(traj.run || runtime < t_end)
    if length(error) > plot_size; error(1) = []; end
    error(end+1) = norm(traj.des(1,1:3)-robot.x(1:3));
    [traj, des] = traj.next(robot.x);
    robot = robot.PID_pos_control(des,yaw,dt);
    
    visualization(robot.transform(), robot.thrust_vector(), traj.des, error, runtime);
    
    pause(0.0001)
    runtime = runtime + 1;
end
fprintf("Simulation end\n");