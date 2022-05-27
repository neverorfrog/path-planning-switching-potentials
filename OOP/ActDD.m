classdef ActDD < Act
    
    methods
        
        %% Constructor (from superclass)
        function obj = ActDD(robot,grid)
            obj = obj@Act(robot,grid);
        end
        
        %% Override of move
        function move(obj,gradX,gradY,tspan)
        end
    end
    
    
end
