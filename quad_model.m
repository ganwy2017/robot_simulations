classdef quad_model
    % x  = [x y z phi theta yaw xd yd zd wx wy wz]'
    %       1 2 3  4    5    6  7  8  9  10 11 12
    properties
        x
        m, g
        Ixx, Iyy, Izz
        u_n
        lin
    end
    
    methods
        function obj = quad_model(m, Ixx, Iyy, Izz, u_n)
            obj.x = zeros(1,12);
            obj.g = 9.81;
            obj.m = m;
            obj.Ixx = Ixx;
            obj.Iyy = Iyy;
            obj.Izz = Izz;
            obj.u_n = u_n;
            obj.lin = pi/12;
        end
        
        function obj = input(obj,u,dt)
            %u = thrust, roll, pitch, yaw
            u = u + randn*sqrt(obj.u_n);
            x_dot(1:3) = obj.x(7:9);                                                                                    % x y z
            x_dot(4)   = obj.x(10) + sin(obj.x(4))*tan(obj.x(5))*obj.x(11) + cos(obj.x(4))*tan(obj.x(5))*obj.x(12);     % phi
            x_dot(5)   = obj.x(11)*cos(obj.x(4))-obj.x(12)*sin(obj.x(4));                                               % theta
            x_dot(6)   = obj.x(11)*sin(obj.x(4))*sec(obj.x(5))+obj.x(12)*cos(obj.x(4))*sec(obj.x(5));                   % yaw
            x_dot(7)   = 1/obj.m*(cos(obj.x(4))*cos(obj.x(6))*sin(obj.x(5))+sin(obj.x(4))*sin(obj.x(6)))*u(1);          % x_dot
            x_dot(8)   = 1/obj.m*(cos(obj.x(4))*sin(obj.x(6))*sin(obj.x(5))-sin(obj.x(4))*cos(obj.x(6)))*u(1);          % y_dot
            x_dot(9)   = -obj.g + 1/obj.m*(cos(obj.x(5))*cos(obj.x(4)))*u(1);                                           % z_dot
            x_dot(10)  = (obj.Iyy-obj.Izz)/obj.Ixx*obj.x(11)*obj.x(12)+1/obj.Ixx*u(2);                                  % wx
            x_dot(11)  = (obj.Izz-obj.Ixx)/obj.Iyy*obj.x(10)*obj.x(12)+1/obj.Iyy*u(3);                                  % wy
            x_dot(12)  = (obj.Ixx-obj.Iyy)/obj.Izz*obj.x(10)*obj.x(11)+1/obj.Izz*u(4);                                  % wz
            
            obj.x = obj.x + x_dot*dt;
        end
        
        function obs = observe(obj, st, noise)
            obs = (obj.x+randn*sqrt(noise)).*st;
        end
        
        function pts = transform(obj)
            l = 0.5;
            pts = [-l  0 0;
                    l  0 0;
                    0 -l 0;
                    0  l 0];
                
            trans = [obj.x(1)*ones(7,1) obj.x(2)*ones(7,1) obj.x(3)*ones(7,1)];
            
            Rx = [1 0 0;
                  0 cos(obj.x(4)) sin(obj.x(4));
                  0 -sin(obj.x(4)) cos(obj.x(4))];
            
            Ry = [cos(obj.x(5)) 0 -sin(obj.x(5));
                  0 1 0;
                  sin(obj.x(5)) 0 cos(obj.x(5))];
                  
            Rz = [cos(obj.x(6)) sin(obj.x(6)) 0;
                  -sin(obj.x(6)) cos(obj.x(6)) 0;
                  0 0 1];
              
            pts = pts*Rx*Ry*Rz;
            
            pts = [pts(1,:); 
                   zeros(1,3);
                   pts(2,:); 
                   zeros(1,3);
                   pts(3,:); 
                   zeros(1,3);
                   pts(4,:)] + trans; 
        end
        
        function pts = thrust_vector(obj)
            l = 0.1;
            pts = [0  0 l];
                
            trans = [obj.x(1)*ones(2,1) obj.x(2)*ones(2,1) obj.x(3)*ones(2,1)];
            
            Rx = [1 0 0;
                  0 cos(obj.x(4)) sin(obj.x(4));
                  0 -sin(obj.x(4)) cos(obj.x(4))];
            
            Ry = [cos(obj.x(5)) 0 -sin(obj.x(5));
                  0 1 0;
                  sin(obj.x(5)) 0 cos(obj.x(5))];
                  
            Rz = [cos(obj.x(6)) sin(obj.x(6)) 0;
                  -sin(obj.x(6)) cos(obj.x(6)) 0;
                  0 0 1];
              
            pts = pts*Rx*Ry*Rz;
            pts = [pts; zeros(1,3)] + trans; 
        end
        
        function obj = PID_att_control(obj,des,T,dt)
            % k = [kpr kpp kpy
            %      kdr kdp kdy]
            k = [2.9 2.9 0.8 1.02 1.02 0.1];
            % st = x   y   z   r   p   y
            %      x_d y_d z_d r_d p_d y_d
            st = [0 0 0 1 1 1 0 0 0 1 1 1];
            %noisy = [0 0 0 0.1 0.1 0.1 0 0 0 0.1 0.1 0.1];
            truth = zeros(1,12);
            obs = obj.observe(st,truth);
            e = k.*([des 0 0 0] - [obs(4:6) obs(10:12)]);
            u = [T e(1)+e(4) e(2)+e(5) e(3)+e(6)];
            u(1) = max(u(1),0);
            obj = obj.input(u,dt);
        end
        
        function obj = PID_pos_control(obj,des,yaw,dt)
            % k = [kpx kpy kpz
            %      kdx kdy kdz]
            k = [50.8 50.8 8.5 43.6  43.6  4.51];
            st = [1 1 1 0 0 0 1 1 1 0 0 0];
            R = [cos(obj.x(6)) sin(obj.x(6));
                 -sin(obj.x(6)) cos(obj.x(6))];
            %noisy = [0.1 0.1 0.1 0 0 0 0.1 0.1 0.1 0 0 0];
            truth = zeros(1,12);
            obs = obj.observe(st,truth);
            e = k.*(des - [obs(1:3) obs(7:9)]);
            u = [e(1)+e(4) e(2)+e(5) e(3)+e(6)];
            u = [u(1:2)*R u(3)];
            u = [-u(2) u(1) u(3)];
            u(1:2) = max(min(u(1:2),obj.lin),-obj.lin);
            obj = obj.PID_att_control([u(1:2),yaw],u(3)+obj.m*obj.g,dt);
        end
        
        
        % sensor readings
        function data = imu(obj)
            % returns angle, angular velocity
            noise = [0.1 0.1 0.1 0.2 0.2 0.2];
            data = [obj.x(4:6) obj.x(10:12)] + randn*sqrt(noise);
            
        end
        
        function data = gps(obj)
            % returns x, y position
            noise = [1 1];
            data = obj.x(1:2) + randn*sqrt(noise);
        end
        
        function data = vio(obj)
            % returns x, y, z, r, p, y
            noise = [0.2 0.2 0.2 0.1 0.1 0.1];
            data = obj.x(1:6) + randn*sqrt(noise);
        end
        
        % ekf
        function x_hat = ekf(obj)
            x_hat = obj.x;
        end
    end
end

