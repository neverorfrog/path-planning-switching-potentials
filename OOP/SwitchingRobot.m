classdef SwitchingRobot < Robot
    
    methods
        function obj = SwitchingRobot(xc,yc,grid)
            obj@Robot(xc,yc,grid);
            obj.sense = Sense(grid);
            obj.plan = SwitchingPlan(grid);
            obj.act = Act(grid);
        end
        
        function obj = start(obj)
            %Plotting data
            figure; axis equal; axis([-0.5 10.5 -0.5 10.5]); axis manual; hold on;
            plot(obj.grid.goal(1),obj.grid.goal(2),"og","linewidth",2);
            %samples = 0;
            %Simulation data
            e = norm([obj.xc,obj.yc]-obj.grid.goal); tspan = 0.05; tsim = 0;
            %Starting simulation
            while(e > 0.1 && tsim < 15)
                obj.iteration(tspan);
                e = norm([obj.xc,obj.yc]-obj.grid.goal); tsim = tsim + tspan; pause(0);
                %Creazione sequenza png della figura
                %samples = samples + 1;
                %filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/simulazione1/pic%d.png', samples);
                %saveas(gcf, filename);
            end
        end
    end
end


