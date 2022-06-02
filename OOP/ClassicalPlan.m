classdef ClassicalPlan < handle
    
    properties
        grid;
        potential; gradX; gradY;
    end
    
    methods
        
        function obj = ClassicalPlan(grid)
            obj.grid = grid;
            obj.totalPotential();
        end
        
        function obj = totalPotential(obj)
            %Attractive potential
            fatt = 1/2*( (obj.grid.X - obj.grid.goal(1)).^2 + (obj.grid.Y - obj.grid.goal(2)).^2 );
            agradX = obj.grid.goal(1)-obj.grid.X; agradY = obj.grid.goal(2)-obj.grid.Y;
            %Repulsive potential
            rgradX = zeros(obj.grid.nc); rgradY = zeros(obj.grid.nc); frep = zeros(obj.grid.nc);
            for i = 1 : length(obj.grid.obstacles)
                %Distanza rispetto agli ostacoli
                dx = abs(obj.grid.X - obj.grid.obstacles(i).xc);
                dy = abs(obj.grid.Y - obj.grid.obstacles(i).yc);
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
            obj.gradX = rgradX + agradX;
            obj.gradY = rgradY + agradY;
            obj.potential = frep + fatt;
        end
    end
end
