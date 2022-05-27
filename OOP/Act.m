classdef Act
    
    properties
        robot; grid;
    end
    
    methods
        %% Constructor
        function obj = Act(robot,grid)
            obj.robot = robot;
            obj.grid = grid;
        end
        
        %% Method that moves the robot according to the desired velocity (Runge-Kutta 2 for integration)
        function move(obj,gradX,gradY,tspan)
            rx = obj.robot.xc;
            ry = obj.robot.yc;
            rtheta = obj.robot.theta;
            [vr,wr] = obj.commands(rx,ry,rtheta,tspan,gradX,gradY);
            Xdot = obj.moveDD(vr,wr,rtheta);
            
            rx2 = rx + tspan/2*Xdot(1);
            ry2 = ry + tspan/2*Xdot(2);
            rtheta2 = rtheta + tspan/2*Xdot(3);
            [vr,wr] = obj.commands(rx2,ry2,rtheta2,tspan,gradX,gradY);
            Xdot = obj.moveDD(vr,wr,rtheta2);
            
            obj.robot.xc = rx + tspan*Xdot(1);
            obj.robot.yc = ry + tspan*Xdot(2);
            obj.robot.theta = rtheta + tspan*Xdot(3);
            
            obj.robot.draw();
        end
        
        %%
        function [vr,wr] = commands(obj,rx,ry,rtheta,tspan,gradX,gradY)
            i = obj.grid.coord2index([rx,ry]);
            thetaN = atan2(gradY(i(2),i(1)),gradX(i(2),i(1)));
            thetaDiff = atan2(sin(thetaN-rtheta),cos(thetaN-rtheta));
            vgrad = [gradX(i(2),i(1)) gradY(i(2),i(1))];
            Mv = norm(vgrad);
            vr = (Mv * cos(thetaDiff));
            
            rx1 = rx + vgrad(1)*tspan;
            ry1 = ry + vgrad(2)*tspan;
            j = obj.grid.coord2index([rx1,ry1]);
            thetaN1 = atan2(gradY(j(2),j(1)),gradX(j(2),j(1)));
            
            thetaDdiff = atan2(sin(thetaN1-thetaN),cos(thetaN1-thetaN));
            thetaDdot = thetaDdiff/tspan;
            Kc = 10; eps = 0.001; v = 1;
            Kw = (thetaDdot + Kc * abs(thetaDiff)^v * sign(thetaDiff))/(thetaDiff+eps);
            wr = (abs(thetaDiff) >= eps) * Kw * (thetaDiff);
        end
        
        function Xdot = moveDD(~,vr,wr,theta)
            % Modello specifico del Differential Drive
            r = 0.05; %raggio delle ruote di 5 centimetri
            L = 0.15; %distanza tra le due ruote di 15 centimetri
            %K rende possibile esprimere vr e wr in funzione delle due velocitá impresse alle ruote
            K = [r/2 r/2 ; r/L -r/L];
            wRwL = K \ [vr ; wr];
            Xdot = ([cos(theta) 0 ; sin(theta) 0 ; 0 1] * K * wRwL);
        end
    end
end
