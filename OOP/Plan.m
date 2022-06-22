classdef (Abstract) Plan < handle
    
    properties
        directive;
        grid;
    end
    
    methods
        function obj = setGrid(grid)
            obj.grid = grid;
        end
    end
    
    methods(Abstract)
        obj = decide(robot,dObstacle);
    end
end
