classdef RealBypassing < Bypassing
    
    properties
        virtualbypass VirtualBypassing;
    end
    
    methods
        function obj = RealBypassing(virtualbypass)
            obj.virtualbypass = virtualbypass;
            obj.gradX = virtualbypass.gradXO;
            obj.gradY = virtualbypass.gradYO;
            obj.obstacle = virtualbypass.obstacle;
        end
        
        function obj = decision(obj,robot,dObstacle)
            decision@Bypassing(obj,robot,dObstacle);
            goal = robot.grid.goal; 
            P2 = obj.virtualbypass.P2; 
            pose = robot.getPose();
            distp2 = norm(goal-P2);
            distpose = norm(goal-[pose(1) pose(2)]);
            if norm([robot.xc robot.yc] - obj.virtualbypass.P2) < 0.1 || distpose < distp2
                robot.setState(robot.attractive);
            end
        end
    end
end
