classdef Sense
    
    methods
        %% Method that looks in a radius rv and a tube T(t) if there are any obstacles
        function dObstacle = scan(obj,robot)
            rx = robot.xc; ry = robot.yc; rv = robot.rv;
            n = length(robot.grid.obstacles);
            distances = zeros(1,n) + inf;
            dObstacles = cell(1,n);
            tube = obj.tube(robot);
            for j = 1 : n
                o = robot.grid.obstacles(j);
                distance = norm([rx ry] - [o.xc o.yc]);
                if distance <= rv && obj.intube(tube,o)
                    dObstacles{j} = o;
                    distances(j) = distance;
                end
            end
            %Ostacolo tra quelli rilevati a distanza minima
            [~,pos] = min(distances);
            dObstacle = dObstacles(pos); dObstacle = dObstacle{1};
        end
    end
    
    methods(Access=private)
        %% Method that builds the tube T(t)
        function tube = tube(~,robot)
            pose = robot.getPose(); rx = pose(1); ry = pose(2);
            grid = robot.grid;
            G = grid.goal;
            rm = 2.5;
            angle = atan2(G(2)-ry,G(1)-rx);
            
            deltaX = rm/2*sin(angle); deltaY = rm/2*cos(angle);
            x1 = rx + deltaX; y1 = ry - deltaY;
            x4 = rx - deltaX; y4 = ry + deltaY;
            x2 = G(1) + deltaX; y2 = G(2) - deltaY;
            x3 = G(1) - deltaX; y3 = G(2) + deltaY;
            tube.eps = 0.1;
            
            tube.m14 = (y4-y1)/(x4-x1); tube.q14 = tube.m14*x1 - y1;
            tube.m12 = (y2-y1)/(x2-x1); tube.q12 = tube.m12*x1 - y1;
            tube.m34 = (y4-y3)/(x4-x3); tube.q34 = tube.m34*x3 - y3;
            
            %Plotting
            persistent plotobj1; persistent plotobj2; persistent plotobj3;
            delete(plotobj1); delete(plotobj2); delete(plotobj3);
            plotobj1 = plot([x1,x2,],[y1,y2],"b"); 
            plotobj2 = plot([x4,x3],[y4,y3],"b");
            plotobj3 = plot([x4,x1],[y4,y1],"b");
            
            tube.x1 = x1; tube.x4 = x4; tube.angle = angle;
        end
        
        function result = intube(~,tube,obstacle)
            if abs(tube.m14) > exp(10)
                v14 = (obstacle.xc - tube.x1);
            else
                v14 = (obstacle.yc - tube.m14*obstacle.xc + tube.q14);
            end
            
            if abs(tube.m12) > exp(10)
                v12 = (tube.x1 - obstacle.xc);
            else
                v12 = (obstacle.yc - tube.m12*obstacle.xc + tube.q12);
            end
            
            if abs(tube.m34) > exp(10)
                v34 = (tube.x4 - obstacle.xc);
            else
                v34 = (obstacle.yc - tube.m34*obstacle.xc + tube.q34);
            end
            
            angle = tube.angle;
            if (angle >= 0 && angle <= pi/2) %first quadrant
                result1 = v14 > eps;
                result2 = v12 > 0; result3 = v34 < 0;
            elseif (angle > pi/2 && angle < pi) %second quadrant
                result1 = v14 > eps;
                result2 = v12 < 0; result3 = v34 > 0;
            elseif (angle >= -pi && angle <= -pi/2 || angle == pi ) %third quadrant
                result1 = v14 < -eps;
                result2 = v12 < 0; result3 = v34 > 0;
            else %fourth quadrant
                result1 = v14 < -eps;
                result2 = v12 > 0; result3 = v34 < 0;
            end
            result = result1 && result2 && result3;
        end
    end
end
