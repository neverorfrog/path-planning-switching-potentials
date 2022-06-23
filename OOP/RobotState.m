classdef (Abstract) RobotState < handle
    
    methods (Abstract)
        obj = operation(obj,robot,dObstacle);
    end
end
