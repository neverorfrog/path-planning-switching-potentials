classdef (Abstract) RobotState < handle
    
    methods (Abstract)
        obj = decision(obj,robot,dObstacle);
    end
end
