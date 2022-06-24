classdef Attractive < RobotState
    methods
        function obj = decision(obj,robot,dObstacle)
            if ~isempty(dObstacle)
                bypassing = VirtualBypassing(robot,dObstacle);
                bypassing.obstacle = [dObstacle.xc dObstacle.yc];
                robot.setState(bypassing);
            end
        end
    end
end
