classdef SwitchingPlan < Plan
    
    properties
        state RobotState;
        attractive Attractive;
    end
    
    methods
        %% Genera la direttiva
        function obj = decide(obj,robot,dObstacle)
            obj.state.operation(robot,dObstacle);
        end
        
        function obj = setState(state)
            state.entryAction(robot);
            obj.state = state;
        end
    end
    
end
