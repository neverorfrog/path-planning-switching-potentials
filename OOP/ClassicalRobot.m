classdef ClassicalRobot < Robot
    
    properties
        path;
        potential;
        act APFAct;
        state;
        tspan;
    end
    
    methods
        function obj = ClassicalRobot(R,L,grid,tspan)
            obj@Robot(R,L,grid);
            obj.act = APFAct();
            obj.totalPotential();
            obj.tspan = tspan;
        end
        
        function initializeFigure(obj)
            obj.initializeFigure@Robot()
            figure(1); quiver(obj.grid.X,obj.grid.Y,obj.state.gradX,obj.state.gradY);
            figure(2); mesh(obj.potential); view([-186.2 31.3]); hold on;
        end
        
        function plotobj = draw(obj,~)
            figure(1); obj.draw@Robot();
            rp = obj.grid.coord2index([obj.xc,obj.yc]);
            z = obj.potential(rp(1),rp(2));
            figure(2); scatter3(rp(2),rp(1),z,"filled","r","linewidth",3);
        end
        
        function obj = start(obj)
            %Figure initialiazation
            obj.initializeFigure();
            %Simulation data
            e = norm([obj.xc,obj.yc]-obj.grid.goal);
            tsim = 0; samples = 0;
            %Starting simulation
            while(e > 0.1 && tsim < 10)
                %Giving the command to the actuators 
                [obj.xc,obj.yc,obj.theta] = obj.act.move(obj,obj.tspan);
                %Plotting
                obj.draw();
                %Moving obstacles
                obj.grid.moveObstacles(obj.tspan);
                %Refreshing the error
                e = norm([obj.xc,obj.yc]-obj.grid.goal); tsim = tsim + obj.tspan; pause(0);
                %Creazione sequenza png della figura
                samples = samples + 1; obj.pngSequence(samples);
            end
        end
        
        function obj = totalPotential(obj)
            %Attractive potential
            fatt = 1/2*( (obj.grid.X - obj.grid.goal(1)).^2 + (obj.grid.Y - obj.grid.goal(2)).^2 );
            agradX = obj.grid.goal(1)-obj.grid.X; agradY = obj.grid.goal(2)-obj.grid.Y;
            %Repulsive potential
            rgradX = zeros(obj.grid.nc); rgradY = zeros(obj.grid.nc); frep = zeros(obj.grid.nc);
            for i = 1 : length(obj.grid.obstacles)
                %Distanza rispetto agli ostacoli
                xo = obj.grid.obstacles(i).xc;
                yo = obj.grid.obstacles(i).yc;
                d = sqrt((obj.grid.x - xo).^2 + (obj.grid.y - yo).^2);
                %Potenziale
                freptemp = 1/2*((1./d - 1/2).^2);
                freptemp(d > 2) = 0;
                freptemp(freptemp > 30) = 5;
                frep = frep + freptemp;
                %Antigradiente
                [rgradXtemp, rgradYtemp] = gradient(-frep,1/1000);
                %Lo sommo ai precedenti
                rgradX = rgradX + rgradXtemp;
                rgradY = rgradY + rgradYtemp;
            end
            obj.state.gradX = rgradX + agradX;
            obj.state.gradY = rgradY + agradY;
            obj.potential = frep + fatt;
            figure;
            quiver(obj.grid.X,obj.grid.Y,obj.state.gradX,obj.state.gradY);
        end
        
        
        function pngSequence(~,samples)
            %Creazione sequenza png della figura 1
            figure(1);
            filename = sprintf("SwitchingPotentials/Latex/simulazioni/minimiLocaliMesh/snap%d.png", samples);
            exportgraphics(gca,filename,'Resolution',300)
            %Creazione sequenza png della figura 2
            figure(2);
            filename = sprintf("SwitchingPotentials/Latex/simulazioni/minimiLocaliQuiver/snap%d.png", samples);
            exportgraphics(gca,filename,'Resolution',300)
        end
    end
end
