classdef Act
    
    properties
        grid;
    end
    
    methods
        %% Constructor
        function obj = Act(grid)
            obj.grid = grid;
        end
        
        %% Metodo che genera velocita lineare e angolare per il robot
        function [vr,wr] = commands(obj,rx,ry,rtheta,tspan,directive)
            gradX = directive.gradX; gradY = directive.gradY;
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
        
        function [Xdot,wRwL] = ode(~,vr,wr,theta,R,L)
            %K rende possibile esprimere vr e wr in funzione delle
            %due velocita impresse alle ruote
            K = [R/2 R/2 ; R/L -R/L];
            wRwL = K \ [vr ; wr];
            Xdot = ([cos(theta) 0 ; sin(theta) 0 ; 0 1] * K * wRwL);
        end
    end   
end
