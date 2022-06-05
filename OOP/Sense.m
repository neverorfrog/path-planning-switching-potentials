classdef Sense
    properties
        grid;
    end
    
    methods
        function obj = Sense(grid)
            obj.grid = grid;
        end
        
        %% Method that looks in a radius rv and a tube T(t) if there are any obstacles
        function [dObstacle] = scan(obj,pose,rv)
            rx = pose(1); ry = pose(2);
            n = length(obj.grid.obstacles);
            distances = zeros(1,n) + inf;
            dObstacles = cell(1,n);
            for j = 1 : n
                o = obj.grid.obstacles(j);
                distance = norm([rx ry] - [o.xc o.yc]);
                if distance <= rv && obj.tube(pose,o)
                    dObstacles{j} = o;
                    distances(j) = distance;
                end
            end
            %Ostacolo tra quelli rilevati a distanza minima
            [~,pos] = min(distances);
            dObstacle = dObstacles(pos); dObstacle = dObstacle{1};
        end
        
        %% Method that builds the tube T(t)
        function result = tube(obj,pose,obstacle)
            rx = pose(1); ry = pose(2);
            G = obj.grid.goal;
            
            rm = 3.5;
            angle = atan2(G(2)-ry,G(1)-rx);
            deltaX = rm/2*sin(angle); deltaY = rm/2*cos(angle);
            x1 = rx + deltaX; y1 = ry - deltaY;
            x4 = rx - deltaX; y4 = ry + deltaY;
            x2 = G(1) + deltaX; y2 = G(2) - deltaY;
            x3 = G(1) - deltaX; y3 = G(2) + deltaY;
            
            m14 = (y4-y1)/(x4-x1); q14 = m14*x1 - y1;
            if abs(m14) > exp(10)
                v14 = (obstacle.xc - x1);
            else
                v14 = (obstacle.yc - m14*obstacle.xc + q14);
            end
            
            m12 = (y2-y1)/(x2-x1); q12 = m12*x1 - y1;
            if abs(m12) > exp(10)
                v12 = (x1 - obstacle.xc);
            else
                v12 = (obstacle.yc - m12*obstacle.xc + q12);
            end
            
            m34 = (y4-y3)/(x4-x3); q34 = m34*x3 - y3;
            if abs(m34) > exp(10)
                v34 = (x4 - obstacle.xc);
            else
                v34 = (obstacle.yc - m34*obstacle.xc + q34);
            end
            
            if (angle >= 0 && angle <= pi/2) %first quadrant
                result1 = v14 >= 0;
                result2 = v12 >= 0; result3 = v34 <= 0;
            elseif (angle > pi/2 && angle < pi) %second quadrant
                result1 = v14 >= 0;
                result2 = v12 <= 0; result3 = v34 >= 0;
            elseif (angle >= -pi && angle <= -pi/2 || angle == pi ) %third quadrant
                result1 = v14 <= 0;
                result2 = v12 <= 0; result3 = v34 >= 0;
            else %fourth quadrant
                result1 = v14 <= 0;
                result2 = v12 >= 0; result3 = v34 <= 0;
            end
            result = result1 && result2 && result3;
        end
    end
end
