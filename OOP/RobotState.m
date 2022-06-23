classdef (Abstract) RobotState < handle
    
    properties
        gradX; gradY;
    end
    
    methods (Abstract)
        obj = decision(obj,robot,dObstacle);
    end
end
