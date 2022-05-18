classdef Obstacle < matlab.mixin.Copyable
    
    %Object that represents an obstacle of circular shape
    
    properties %wrt to its position and movement
        id; %serial number
        xc; yc; %position
        theta; %orientation
        raggio; %size
        v; %velocity
        plotobj; %graphic handle
    end
    
    properties %wrt its bypassing potential
        gx; gy; %antigradient associated to the bypassing potential
        sense; %bypassing sense (clockwise or counterclockwise)
        c; %bypassing antigradient modulation
        bypassed;
    end
    
    methods
        
        function obj = Obstacle(xc,yc,v)
            obj.id = randi(10000);
            obj.xc = xc;
            obj.yc = yc;
            obj.raggio = 0.5;
            obj.v = v;
            obj.theta = atan2(v(2),v(1));
            obj.bypassed = true;
        end
        
        function plotobj = draw(obj)
            r = obj.raggio; x = obj.xc; y = obj.yc; t = 0:0.01:2*pi;
            delete(obj.plotobj);
            plotobj = plot(cos(t)*r+x,sin(t)*r+y,"r","linewidth",2); hold on;
            plot(obj.xc,obj.yc,"or");
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
        
        function [gx,gy] = antigradient(obj,X,Y,c)
            %Calcolo del gradiente
            obj.c = c; 
            obj.gx = obj.c*(Y(:,1)-obj.yc)./((X(1,:)-obj.xc).^2+(Y(:,1)-obj.yc).^2+0.00001);
            obj.gy = obj.c*(obj.xc-X(1,:))./((X(1,:)-obj.xc).^2+(Y(:,1)-obj.yc).^2+0.00001);
            if obj.sense == "counterclock"
                obj.gx = -obj.gx; obj.gy = -obj.gy;
            end
            gx = obj.gx; gy = obj.gy;
        end
        
        function chooseSense(obj,r)
            phi = atan2(obj.yc - r.yc,obj.xc - r.xc);
            alphav = atan2(obj.v(2),obj.v(1));
            vphi = atan2(sin(-phi + pi/2 + alphav),cos(-phi + pi/2 + alphav));
            
            if obj.v(1) == 0 && obj.v(2) == 0
                if phi - r.theta > 0
                    obj.sense = "counterclock";
                else
                    obj.sense = "clock";
                end
            else
                if cos(vphi) > 0
                    obj.sense = "clock";
                else
                    obj.sense = "counterclock";
                end
            end
        end
        
    end
    
end
