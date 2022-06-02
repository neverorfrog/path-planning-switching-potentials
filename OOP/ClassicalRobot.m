classdef ClassicalRobot < Robot
    
    properties
        path;
    end
    
    methods
        function obj = ClassicalRobot(xc,yc,grid)
            obj@Robot(xc,yc,grid);
            obj.plan = ClassicalPlan(grid);
            obj.act = Act(grid);
        end
        
        function obj = start(obj)
            %Plotting data
            %quiver(obj.grid.X,obj.grid.Y,obj.plan.gradX,obj.plan.gradY);
            mesh(obj.plan.potential); view([-122.4 43.8]); hold on;
            %samples = 0;
            %Simulation data
            e = norm([obj.xc,obj.yc]-obj.grid.goal); tspan = 0.05; tsim = 0;
            %Starting simulation
            while(e > 0.1 && tsim < 15)
                pose = obj.getPose();
                rx = pose(1); ry = pose(2);
                rp = obj.grid.coord2index([rx,ry]);
                z = obj.plan.potential(rp(1),rp(2));
                scatter3(rp(1),rp(2),z,"filled","linewidth",3);
                obj.path = [obj.path ; [rp(2) rp(1) z]];
                newPose = obj.act.move(pose,obj.plan.gradX,obj.plan.gradY,tspan);
                obj.xc = newPose(1); obj.yc = newPose(2); obj.theta = newPose(3);
                e = norm([obj.xc,obj.yc]-obj.grid.goal); tsim = tsim + tspan; pause(0);
                %Creazione sequenza png della figura
                %samples = samples + 1;
                %filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/simulazione1/pic%d.png', samples);
                %saveas(gcf, filename);
            end
        end
    end
end
