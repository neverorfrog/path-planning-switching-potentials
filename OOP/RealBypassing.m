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
            if norm([robot.xc robot.yc] - obj.virtualbypass.P2) < 0.1
                robot.setState(robot.attractive);
            end
        end
    end
end
