classdef Paraboloidal < Attractive
    
    methods
        function obj = Paraboloidal(grid)
            obj.gradX = grid.goal(1)-grid.X;
            obj.gradY = grid.goal(2)-grid.Y;
        end
        
         function obj = decision(obj,robot,dObstacle)
            decision@Attractive(obj,robot,dObstacle);
        end
    end
end
