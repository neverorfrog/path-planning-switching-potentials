classdef ClassicalRobot < Robot
    
    methods
        function obj = ClassicalRobot(xc,yc,grid)
            obj@Robot(xc,yc,grid);
            obj.plan = ClassicalPlan(grid);
            obj.act = Act(grid);
        end
        
        function [gradX,gradY] = totalPotential(obj)
            grid = obj.grid;
            %Attractive potential
            fatt = 1/2*( (grid.X - grid.goal(1)).^2 + (grid.Y - grid.goal(2)).^2 );
            agradX = grid.goal(1)-grid.X; agradY = grid.goal(2)-grid.Y;
            %Repulsive potential
            rgradX = zeros(grid.nc); rgradY = zeros(grid.nc); frep = zeros(grid.nc);
            for i = 1 : n
                %Distanza rispetto agli ostacoli
                dx = abs(grid.X - grid.obstacles(i).xc);
                dy = abs(grid.Y - grid.obstacles(i).yc);
                d = sqrt(dx.^2 + dy.^2);
                %Potenziale
                freptemp = 1/2*((1./d - 1/2).^2);
                freptemp(d > 2) = 0;
                freptemp(isinf(freptemp)) = 10;
                frep = frep + freptemp;
                %Antigradiente
                [rgradXtemp, rgradYtemp] = gradient(-frep,1/1000);
                %Lo sommo ai precedenti
                rgradX = rgradX + rgradXtemp;
                rgradY = rgradY + rgradYtemp;
            end
            gradX = rgradX + agradX;
            gradY = rgradY + agradY;
            potential = frep + fatt;
        end
    end
end
