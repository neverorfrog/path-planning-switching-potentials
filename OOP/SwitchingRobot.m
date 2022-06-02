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
            f = figure; axis equal; axis([-0.5 10.5 -0.5 10.5]); axis manual; hold on;
            plot(obj.grid.goal(1),obj.grid.goal(2),"og","linewidth",2);
            %Simulation data
            e = norm([obj.xc,obj.yc]-obj.grid.goal); tspan = 0.05; tsim = 0; samples = 0;
            %Starting simulation
            while(e > 0.1 && tsim < 15)
                %Current pose
                pose = [obj.xc obj.yc obj.theta];
                %Control loop
                dObstacle = obj.sense.scan(pose,obj.rv);
                obj.plan.decide(dObstacle,pose);
                newPose = obj.act.move(obj.plan.gradX,obj.plan.gradY,tspan,pose);
                obj.xc = newPose(1); obj.yc = newPose(2); obj.theta = newPose(3);
                obj.draw(true);
                %Moving obstacles 
                for k = 1 : length(obj.grid.obstacles)
                    obj.grid.obstacles(k).move(tspan);
                end
                %Paraboloidal potential
%                 if e <= 1 && e >= 0.9
%                     agradX = obj.grid.goal(1)-obj.grid.X; 
%                     agradY = obj.grid.goal(2)-obj.grid.Y;
%                     obj.plan.setGrad(agradX,agradY);
%                 end
                %Simulation data
                e = norm([obj.xc,obj.yc]-obj.grid.goal); tsim = tsim + tspan; pause(0);
                %Creazione sequenza png della figura
%                 samples = samples + 1;
%                 filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/simulazione1/pic%d.png', samples);
%                 saveas(gcf, filename);
            end
        end
        
        
    end
end
