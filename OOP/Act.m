classdef Act
    
    properties
        robot; grid;
    end
    
    methods
        %% Constructor
        function obj = Act(grid)
            obj.grid = grid;
        end
        
        %% Method that moves the robot according to the 
        % desired velocity (Runge-Kutta 2 for integration)
%         function newPose = move(obj,pose,gradX,gradY,tspan)
%             rx = pose(1);
%             ry = pose(2);
%             rtheta = pose(3);
%             [vr,wr] = obj.commands(rx,ry,rtheta,tspan,gradX,gradY);
%             Xdot = obj.ode(vr,wr,rtheta);
%             
%             rx2 = rx + tspan/2*Xdot(1);
%             ry2 = ry + tspan/2*Xdot(2);
%             rtheta2 = rtheta + tspan/2*Xdot(3);
%             [vr,wr] = obj.commands(rx2,ry2,rtheta2,tspan,gradX,gradY);
%             Xdot = obj.ode(vr,wr,rtheta2);
%             
%             newPose(1) = rx + tspan*Xdot(1);
%             newPose(2) = ry + tspan*Xdot(2);
%             newPose(3) = rtheta + tspan*Xdot(3);
%         end
        
        %% Metodo che genera velocita lineare e angolare per il robot
        function [vr,wr] = commands(obj,rx,ry,rtheta,tspan,gradX,gradY)
            i = obj.grid.coord2index([rx,ry]);
            thetaN = atan2(gradY(i(1),i(2)),gradX(i(1),i(2)));
            thetaDiff = atan2(sin(thetaN-rtheta),cos(thetaN-rtheta));
            vgrad = [gradX(i(1),i(2)) gradY(i(1),i(2))];
            Mv = norm(vgrad);
            vr = (Mv * cos(thetaDiff));
            
            rx1 = rx + vgrad(1)*tspan;
            ry1 = ry + vgrad(2)*tspan;
            j = obj.grid.coord2index([rx1,ry1]);
            thetaN1 = atan2(gradY(j(1),j(2)),gradX(j(1),j(2)));
            
            thetaDdiff = atan2(sin(thetaN1-thetaN),cos(thetaN1-thetaN));
            thetaDdot = thetaDdiff/tspan;
            Kc = 10; eps = 0.001; v = 1;
            Kw = (thetaDdot + Kc * abs(thetaDiff)^v * sign(thetaDiff))/(thetaDiff+eps);
            wr = (abs(thetaDiff) >= eps) * Kw * (thetaDiff);
        end
    end
    
    methods(Access = private)
        function [Xdot,wRwL] = odeDD(~,vr,wr,theta)
            % Modello specifico del Differential Drive
            r = 0.1; %raggio delle ruote di 10 centimetri
            L = 0.25; %distanza tra le due ruote di 25 centimetri
            %K rende possibile esprimere vr e wr in funzione delle 
            %due velocita impresse alle ruote
            K = [r/2 r/2 ; r/L -r/L];
            wRwL = K \ [vr ; wr];
            Xdot = ([cos(theta) 0 ; sin(theta) 0 ; 0 1] * K * wRwL);
        end
    end
end
