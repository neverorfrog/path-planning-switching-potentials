classdef Conical < Attractive
    methods
        function obj = Conical(grid)
            %Setting potenziale conico
            di = sqrt((grid.goal(1)-grid.X).^2 + (grid.goal(2)-grid.Y).^2);
            obj.gradX = (grid.goal(1)-grid.X)./di; obj.gradY = (grid.goal(2)-grid.Y)./di;
        end
        
        function obj = decision(obj,robot,dObstacle)
            decision@Attractive(obj,robot,dObstacle);
            rx = robot.xc; ry = robot.yc; grid = robot.grid;
            if norm([rx,ry]-grid.goal) < 1
                newattractive = Paraboloidal(grid);
                robot.attractive = newattractive;
                robot.setState(newattractive);
            end
        end
    end
end
