classdef ClassicalRobot < Robot
    
    properties
        path;
    end
    
    methods
        function obj = ClassicalRobot(R,L,grid)
            obj@Robot(R,L,grid);
            obj.plan = ClassicalPlan(grid);
            obj.act = Act(grid);
            obj.sense = NoSense(grid);
        end
        
        function initializeFigure(obj)
            obj.initializeFigure@Robot()
            quiver(obj.grid.X,obj.grid.Y,obj.plan.directive.gradX,obj.plan.directive.gradY);
            figure(2); mesh(obj.plan.potential); view([-186.2 31.3]); hold on;
        end
        
        function plotobj = draw(obj,~)
            figure(1); obj.draw@Robot();
            rp = obj.grid.coord2index([obj.xc,obj.yc]);
            z = obj.plan.potential(rp(1),rp(2));
            figure(2); scatter3(rp(2),rp(1),z,"filled","b","linewidth",3);
        end
        
        function pngSequence(~,samples)
            %Creazione sequenza png della figura 1
            figure(1);
            filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/minimiLocaliQuiver/pic%d.png', samples);
            saveas(gcf, filename);
            %Creazione sequenza png della figura 2
            figure(2); 
            filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/minimiLocaliMesh/pic%d.png', samples);
            saveas(gcf, filename);
        end
    end
end
