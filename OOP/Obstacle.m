classdef Obstacle < handle
    
    %Object that represents an obstacle of circular shape
    
    properties %wrt to its position and movement
        id; %serial number
        xc; %position
        yc; %position
        theta; %orientation
        raggio; %size
        v; %velocity
        plotobj; %graphic handle
    end
    
    methods
        
        function obj = Obstacle(xc,yc,v)
            obj.id = randi(10000);
            obj.xc = xc;
            obj.yc = yc;
            obj.raggio = 0.5;
            obj.v = v;
            obj.theta = atan2(v(2),v(1));
        end
        
        function plotobj = draw(obj)
            r = obj.raggio; x = obj.xc; y = obj.yc; t = 0:0.01:2*pi;
            delete(obj.plotobj);
            plotobj = plot(cos(t)*r+x,sin(t)*r+y,"r","linewidth",2);
%         plot(obj.xc,obj.yc,"or");
            obj.plotobj = plotobj;
        end
        
        function move(obj,tspan)
            movableX = obj.xc + obj.v(1)*tspan >= 0 && obj.xc + obj.v(1)*tspan <= 10;
            movableY = obj.yc + obj.v(2)*tspan >= 0 && obj.yc + obj.v(2)*tspan <= 10;
            if ~movableX && ~movableY
                obj.v(1) = -obj.v(1);
                obj.v(2) = -obj.v(2);
                obj.id = randi(10000);
            elseif ~movableX && movableY
                obj.v(1) = -obj.v(1);
                obj.id = randi(10000);
            elseif ~movableY && movableX
                obj.v(2) = -obj.v(2);
                obj.id = randi(10000);
            end
            obj.xc = obj.xc + obj.v(1)*tspan;
            obj.yc = obj.yc + obj.v(2)*tspan;
            obj.draw();
            
        end 
        
    end
    
end
