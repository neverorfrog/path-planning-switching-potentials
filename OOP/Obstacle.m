classdef Obstacle < handle
    
    %Object that represents an obstacle of circular shape
    
    properties %wrt to its position and movement
        xc; %position
        yc; %position
        theta; %orientation
        raggio; %size
        v; %velocity
        plotobj;
    end
    
    methods
        
        function obj = Obstacle(xc,yc,v)
            obj.xc = xc;
            obj.yc = yc;
            obj.raggio = 0.5;
            obj.v = v;
            obj.theta = atan2(v(2),v(1));
        end
        
        function draw(obj)
            delete(obj.plotobj);
            obstacle = nsidedpoly(2000, 'Center', [obj.xc obj.yc], 'Radius', obj.raggio);
            obj.plotobj = plot(obstacle, 'FaceColor', 'r');
        end
        
        function move(obj,tspan)
            movableX = obj.xc + obj.v(1)*tspan >= 0 && obj.xc + obj.v(1)*tspan <= 10;
            movableY = obj.yc + obj.v(2)*tspan >= 0 && obj.yc + obj.v(2)*tspan <= 10;
            if ~movableX && ~movableY
                obj.v(1) = -obj.v(1);
                obj.v(2) = -obj.v(2);
            elseif ~movableX && movableY
                obj.v(1) = -obj.v(1);
            elseif ~movableY && movableX
                obj.v(2) = -obj.v(2);
            end
            obj.xc = obj.xc + obj.v(1)*tspan;
            obj.yc = obj.yc + obj.v(2)*tspan;
            obj.draw();
        end 
        
    end
    
end
