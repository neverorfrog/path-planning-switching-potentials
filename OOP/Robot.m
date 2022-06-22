classdef (Abstract) Robot < handle
    
    %Object that represents a robot of triangular shape in position (xc,yc) and a certain orientation
    %theta wrt the positive x axis in R2
    
    properties
        xc; yc; theta; %position and orientation
        rv; %vision radius
        plotobj; %graphic handle
        shape; %circular or triangular
        R; L; %differential drive
        grid Grid; %grid representation of the world
    end
    
    properties (Abstract)
        sense Sense;
        plan Plan;
        act Act;
    end
    
    methods
        %% Constructor
        function obj = Robot(R,L,grid)
            obj.xc = 5; obj.yc = 0; obj.theta = pi/2;
            obj.shape = "triangular";
            obj.rv = 1.5;
            obj.grid = grid;
            obj.R = R; obj.L = L;
        end
        
        function pose = getPose(obj)
            pose(1) = obj.xc;
            pose(2) = obj.yc;
            pose(3) = obj.theta;
        end
        
        function initializeFigure(obj)
            figure(1); axis equal; axis([0 obj.grid.width 0 obj.grid.width]); axis manual; hold on;
            plot(obj.grid.goal(1),obj.grid.goal(2),"og","linewidth",2);
        end
        
        %% Graphic representation of the robot
        function draw(obj)
            if obj.shape == "triangular"
                o = pi/2 - atan2(sin(obj.theta),cos(obj.theta)); %orientation
                p = [obj.xc;obj.yc]; %position
                c = [cos(o) sin(o) ; -sin(o) cos(o)] * [0 1 -1 ; 1 -1 -1] * 0.3 + p; %coordinates
                delete(obj.plotobj);
                obj.plotobj = plot(polyshape(c(1,:),c(2,:)),"LineWidth",2,"FaceColor","b");
                plot(obj.xc,obj.yc,"ob");
            elseif obj.shape == "circular"
                r = 0.3; x = obj.xc; y = obj.yc; t = 0:0.01:2*pi;
                delete(obj.plotobj);
                obj.plotobj = plot(cos(t)*r+x,sin(t)*r+y,"b","linewidth",2);
                plot(obj.xc,obj.yc,"ob");
            end
        end
        
        %% Simulation
        function obj = start(obj)
            %Figure initialiazation
            obj.initializeFigure();
            %Simulation data
            e = norm([obj.xc,obj.yc]-obj.grid.goal);
            tspan = 0.05; tsim = 0; samples = 0;
            %Starting simulation
            while(e > 0.1 && tsim < 20)
                %Sensed obstacle (empty array if nothing was sensed)
                dObstacle = obj.sense.scan(obj);
                %New directive
                obj.plan.decide(obj,dObstacle);
                %Giving the command to the actuators
                [obj.xc,obj.yc,obj.theta] = obj.act.move(obj,tspan);
                %Plotting
                obj.draw();
                %Moving obstacles
                obj.grid.moveObstacles(tspan);
                %Refreshing the error
                e = norm([obj.xc,obj.yc]-obj.grid.goal); tsim = tsim + tspan; pause(0);
                %Creazione sequenza png della figura
                %samples = samples + 1; obj.pngSequence(samples);
            end
        end
        
        function [Xdot,wRwL] = ode(obj,vr,wr)
            %K rende possibile esprimere vr e wr in funzione delle
            %due velocita impresse alle ruote
            K = [obj.R/2 obj.R/2 ; obj.R/obj.L -obj.R/obj.L];
            wRwL = K \ [vr ; wr];
            Xdot = ([cos(obj.theta) 0 ; sin(obj.theta) 0 ; 0 1] * K * wRwL);
        end
    end
    
    methods (Abstract)
        pngSequence(obj,samples)
    end
end
