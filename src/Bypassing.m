classdef (Abstract) Bypassing < RobotState
    
    properties
        obstacle; %ostacolo che sta per essere bypassato
    end
    
    methods
        function obj = decision(obj,robot,dObstacle)
            %Controllo se l'ostacolo rilevato e' diverso
            %da quello che sto bypassando
            dO = obj.checkIfSame(robot,dObstacle);
            robot.attractive.decision(robot,dO);
        end
        
        function dO = checkIfSame(obj,robot,dObstacle)
            %%Ostacolo non rilevato
            if isempty(dObstacle)
                obj.obstacle = []; dO = []; return;
            end
            %%Ostacolo rilevato
            %Ho ancora in vista l'ostacolo che sto bypassando
            if ~isempty(obj.obstacle)
                %Controllo se l'ostacolo rilevato e' lo stesso che sto bypassando
                distance = sqrt((obj.obstacle(1) - dObstacle.xc)^2 + ...
                    (obj.obstacle(2) - dObstacle.yc)^2);
                if abs(distance - norm(robot.tspan*(dObstacle.v))) < 0.01
                    dO = []; %SI
                else
                    dO = dObstacle; %NO
                end
            end
            %Non ho piu in vista l'ostacolo che sto bypassando
            if isempty(obj.obstacle)
                dO = dObstacle;
            end
            %Aggiorno l'ostacolo corrente con quello rilevato (vecchio o nuovo che sia)
            obj.obstacle = [dObstacle.xc dObstacle.yc];
        end
    end
end
