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
            figure(1); axis equal; axis([0 obj.grid.width 0 obj.grid.width]); axis manual; hold on;
            quiver(obj.grid.X,obj.grid.Y,obj.plan.gradX,obj.plan.gradY); 
            figure(2); mesh(obj.plan.potential); view([-186.2 31.3]); hold on;
            %samples = 0;
            %Simulation data
            e = norm([obj.xc,obj.yc]-obj.grid.goal); tspan = 0.05; tsim = 0; samples = 0;
            %Starting simulation
            while(e > 0.1 && tsim < 5)
                pose = obj.getPose();
                rx = pose(1); ry = pose(2);
                rp = obj.grid.coord2index([rx,ry]);
                z = obj.plan.potential(rp(1),rp(2));
                samples = samples + 1;
                %%Quiver
                figure(1); obj.draw(true);
                %Creazione sequenza png della figura
                filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/minimiLocaliQuiver/pic%d.png', samples);
                saveas(gcf, filename);
                %%Mesh
                figure(2); scatter3(rp(2),rp(1),z,"filled","b","linewidth",3);
                %Creazione sequenza png della figura
                filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/minimiLocaliMesh/pic%d.png', samples);
                saveas(gcf, filename);
                %Aggiornamento traiettoria
                obj.path = [obj.path ; [rp(2) rp(1) z]];
                newPose = obj.act.move(pose,obj.plan.gradX,obj.plan.gradY,tspan);
                obj.xc = newPose(1); obj.yc = newPose(2); obj.theta = newPose(3);
                e = norm([obj.xc,obj.yc]-obj.grid.goal); tsim = tsim + tspan; pause(0);
            end
        end
    end
end
