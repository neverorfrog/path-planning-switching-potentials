classdef SwitchingRobot < Robot
    
    properties
        act APFAct = APFAct();
        sense Sense = Sense();
        state RobotState = DefaultRobotState();
        attractive Attractive;
        tspan;
    end
    
    methods
        function obj = SwitchingRobot(R,L,grid,tspan)
            obj@Robot(R,L,grid);
            obj.state = Conical(grid);
            obj.attractive = obj.state;
            obj.tspan = tspan;
        end
        
        function obj = setState(obj,state)
            obj.state = state;
        end
        
        function pngSequence(~,samples)
            filename = sprintf("SwitchingPotentials/Latex/simulazioni/moltiostacoli/snap%d.png", samples);
            saveas(gcf, filename);
        end
        
        function obj = start(obj)
            %Figure initialiazation
            obj.initializeFigure();
            %Simulation data
            e = norm([obj.xc,obj.yc]-obj.grid.goal);
            tsim = 0; samples = 0;
            %Starting simulation
            while(e > 0.1 && tsim < 20)
                %Sensed obstacle (empty array if nothing was sensed)
                dObstacle = obj.sense.scan(obj);
                %New directive
                obj.state.decision(obj,dObstacle);
                %Giving the command to the actuators 
                [obj.xc,obj.yc,obj.theta] = obj.act.move(obj,obj.tspan);
                %Plotting
                obj.draw();
                %Moving obstacles
                obj.grid.moveObstacles(obj.tspan);
                %Refreshing the error
                e = norm([obj.xc,obj.yc]-obj.grid.goal); tsim = tsim + obj.tspan; pause(0);
                %Creazione sequenza png della figura
%                 samples = samples + 1; obj.pngSequence(samples);
            end
        end
    end
end
