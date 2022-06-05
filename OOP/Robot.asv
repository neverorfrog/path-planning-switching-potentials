classdef Robot < handle
    
    %Object that represents a robot of triangular shape in position (xc,yc) and a certain orientation
    %theta wrt the positive x axis in R2
    
    properties 
        xc; yc; theta; %position and orientation
        rv; %vision radius
        grid; %grid representation of the world
        plotobj; %graphic handle
        shape; %circular or triangular
    end
    
    properties (Access = protected)
        sense;
        plan;
        act;
    end
    
    methods (Abstract)
        obj = start(obj)
    end
    
    methods
        %% Constructor
        function obj = Robot(xc,yc,grid)
            obj.xc = xc; obj.yc = yc; obj.theta = pi/2;
            obj.shape = "triangular";
            obj.rv = 1.5;
            obj.grid = grid;
        end
        
        function pose = getPose(obj)
            pose(1) = obj.xc;
            pose(2) = obj.yc;
            pose(3) = obj.theta;
        end
        
        %% Graphic representation of the robot
        function plotobj = draw(obj,traj)
            if obj.shape == "triangular"
                o = pi/2 - atan2(sin(obj.theta),cos(obj.theta)); %orientation
                p = [obj.xc;obj.yc]; %position
                c = [cos(o) sin(o) ; -sin(o) cos(o)] * [0 1 -1 ; 1 -1 -1] * 0.3 + p; %coordinates
                delete(obj.plotobj);
                plotobj = plot(polyshape(c(1,:),c(2,:)),"LineWidth",2,"FaceColor","b");
                if traj
                    plot(obj.xc,obj.yc,"ob");
                end
                obj.plotobj = plotobj;
            elseif obj.shape == "circular"
                r = 0.3; x = obj.xc; y = obj.yc; t = 0:0.01:2*pi;
                delete(obj.plotobj);
                plotobj = plot(cos(t)*r+x,sin(t)*r+y,"b","linewidth",2);
                if traj
                    plot(obj.xc,obj.yc,"ob");
                end
                obj.plotobj = plotobj;
            end
        end
        
        %% Actuator interface
        function obj = move(obj,pose,tspan)
            rx = pose(1);
            ry = pose(2);
            rtheta = pose(3);
            [vr,wr] = obj.act.commands(rx,ry,rtheta,tspan,gradX,gradY);
            Xdot = obj.ode(vr,wr,rtheta);
            
            rx2 = rx + tspan/2*Xdot(1);
            ry2 = ry + tspan/2*Xdot(2);
            rtheta2 = rtheta + tspan/2*Xdot(3);
            [vr,wr] = obj.act.commands(rx2,ry2,rtheta2,tspan,gradX,gradY);
            Xdot = obj.ode(vr,wr,rtheta2);
            
            obj.xc = rx + tspan*Xdot(1);
            obj.yc = ry + tspan*Xdot(2);
            obj.theta = rtheta + tspan*Xdot(3);
        end
    end
end
