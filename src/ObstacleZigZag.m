classdef ObstacleZigZag < Obstacle
    
    %Object that represents an obstacle of circular shape
    %moving in a zigzag manner
    
    properties
        turncounter = 0;
    end
    
     methods
        function move(obj)
            if obj.turncounter == 20
                obj.turncounter = 0;
                obj.v(1) = -obj.v(1);
                obj.v(2) = -obj.v(2);
            end
            obj.turncounter = obj.turncounter + 1;
            move@Obstacle(obj);
        end
     end
end
