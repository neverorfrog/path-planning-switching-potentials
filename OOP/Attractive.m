classdef Attractive < RobotState
    
    properties
        paraboloidal; %booleano che indica se siamo giá passati al potenziale paraboloide
        gradX; gradY;
    end
    
    methods
        function obj = Attractive(grid)
            %Setting potenziale conico
            obj.paraboloidal = false;
            di = sqrt((grid.goal(1)-grid.X).^2 + (grid.goal(2)-grid.Y).^2);
            obj.gradX = (grid.goal(1)-grid.X)./di; obj.gradY = (grid.goal(2)-grid.Y)./di;
        end
        
        function obj = operation(obj,robot,dObstacle)
            rx = robot.xc; ry = robot.yc; grid = robot.grid;
            if ~obj.paraboloidal && norm([rx,ry]-grid.goal) < 1
                obj.gradX = grid.goal(1)-grid.X;
                obj.gradY = grid.goal(2)-grid.Y;
            end
            if ~isempty(dObstacle)
                bypassing = Bypassing(robot,dObstacle);
                bypassing.obstacle = [dObstacle.xc dObstacle.yc];
                robot.setState(bypassing);
            end
        end
    end
end
