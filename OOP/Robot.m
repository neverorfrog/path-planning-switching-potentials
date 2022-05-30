classdef Robot < handle
    
    %Object that represents a robot of triangular shape in position (xc,yc) and a certain orientation
    %theta wrt the positive x axis in R2
    
    properties %position
        xc; yc; theta; %position and orientation
        rv; %vision radius
    end
    
    properties %plotting
        plotobj; %graphic handle
        shape; %circular or triangular
    end
   
    
    
    methods
        %% Constructor
        function obj = Robot(xc,yc)
            obj.xc = xc; obj.yc = yc; obj.theta = pi/2;
            obj.shape = "triangular";
            obj.rv = 1.5;
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
    end
end
