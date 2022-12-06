classdef Attractive < RobotState
    methods
        function obj = decision(obj,robot,dObstacle)
            if ~isempty(dObstacle)
                [bypassing,success] = VirtualBypassing(robot,dObstacle);
                if success
                    bypassing.obstacle = [dObstacle.xc dObstacle.yc];
                    robot.setState(bypassing);
                end
            end
        end
    end
end
