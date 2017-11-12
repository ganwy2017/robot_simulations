classdef trajectory  
    properties
        C       % spline parameters
        des     % current desired state
        euD     % completion distance
        t0      % current time in spline
        T       % time per spline
        run     % run state
    end
    
    methods
        function obj = trajectory(euD)
            obj.des = zeros(1,12);
            obj.euD = euD;
            obj.run = false;
        end
        
        function obj = add_spline(obj, x0, xf, T, order)
            
        end
        
        function obj = add_des(obj,des)
            obj.run = true;
            obj.des = des;
        end
        
        function [obj, des] = next(obj,x)
            if norm(obj.des(1,1:3)-x(1:3)) < obj.euD
                if size(obj.des,1) > 1
                    obj.des(1,:) = [];
                end
            end
            des = obj.des(1,:);
        end
    end
end

