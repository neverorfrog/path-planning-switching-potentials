classdef Plan < handle
    
    
    properties
        directive;
        grid;
    end
    
    methods
        function obj = Plan(grid)
            obj.grid = grid;
        end
    end
    
    methods(Abstract)
        obj = decide(obj)
    end
end
