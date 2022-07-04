classdef Robot < handle
    
    %Robot differential drive
    
    properties
        xc; yc; theta; %position and orientation
        rv; %vision radius
        R; L; %differential drive
        grid Grid; %grid representation of the world
    end
    
    methods
        %% Constructor
        function obj = Robot(R,L,grid)
            obj.xc = 4; obj.yc = 0; obj.theta = 1.2;
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
            plot(obj.grid.goal(1),obj.grid.goal(2),"pg","linewidth",2);
            set(gca,"Color","none");
        end
        
        %% Graphic representation of the robot
        function draw(obj)
            persistent plotobj1; persistent plotobj2; 
            delete(plotobj1); delete(plotobj2); 
            o = pi/2 - atan2(sin(obj.theta),cos(obj.theta)); %orientation
            p = [obj.xc;obj.yc]; %position
            c = [cos(o) sin(o) ; -sin(o) cos(o)] * [0 1 -1 ; 1 -1 -1] * 0.2 + p; %coordinates
            plotobj1 = plot(polyshape(c(1,:),c(2,:)),"LineWidth",2,"FaceColor","b");
            plot(obj.xc,obj.yc,"ob");
            visionradius = nsidedpoly(2000, 'Center', [obj.xc obj.yc], 'Radius', obj.rv);
            plotobj2 = plot(visionradius, 'FaceColor', 'c');
        end
        
        function [Xdot,wRwL] = ode(obj,vr,wr)
            %K rende possibile esprimere vr e wr in funzione delle
            %due velocita impresse alle ruote
            K = [obj.R/2 obj.R/2 ; obj.R/obj.L -obj.R/obj.L];
            wRwL = K \ [vr ; wr];
            Xdot = ([cos(obj.theta) 0 ; sin(obj.theta) 0 ; 0 1] * K * wRwL);
        end
    end
end
