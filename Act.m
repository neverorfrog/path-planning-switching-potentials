classdef (Abstract) Act < handle
    
    methods
        %% Actuator interface
        function [xc,yc,theta] = move(obj,robot,tspan)
            rx = robot.xc;
            ry = robot.yc;
            rtheta = robot.theta;
            [vr,wr] = obj.commands(rx,ry,rtheta,tspan,robot);
            Xdot = robot.ode(vr,wr);
            
            rx2 = rx + tspan/2*Xdot(1);
            ry2 = ry + tspan/2*Xdot(2);
            rtheta2 = rtheta + tspan/2*Xdot(3);
            [vr,wr] = obj.commands(rx2,ry2,rtheta2,tspan,robot);
            Xdot = robot.ode(vr,wr);
            
            xc = rx + tspan*Xdot(1);
            yc = ry + tspan*Xdot(2);
            theta = rtheta + tspan*Xdot(3);
        end
    end
    
    methods (Abstract)
        [vr,wr] = commands(~,rx,ry,rtheta,tspan,robot);
    end
end
