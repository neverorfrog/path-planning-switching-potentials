classdef NoSense < Sense
    
    methods
        
        function obj = NoSense(grid)
            obj@Sense(grid)
        end
        
        %% Method that looks in a radius rv and a tube T(t) if there are any obstacles
        function dObstacle = scan(~,~,~)
            dObstacle = [];
        end
        
    end
end
