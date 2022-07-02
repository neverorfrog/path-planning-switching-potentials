classdef APFAct < Act
    
    methods
        %% Metodo che genera velocita lineare e angolare per il robot
        function [vr,wr] = commands(~,rx,ry,rtheta,tspan,robot)
            grid = robot.grid; gradX = robot.state.gradX; gradY = robot.state.gradY;
            i = grid.coord2index([rx,ry]);
            thetaN = atan2(gradY(i(1),i(2)),gradX(i(1),i(2)));
            thetaDiff = atan2(sin(thetaN-rtheta),cos(thetaN-rtheta));
            vgrad = [gradX(i(1),i(2)) gradY(i(1),i(2))];
            Mv = norm(vgrad);
            vr = (Mv * cos(thetaDiff));
            
            rx1 = rx + vgrad(1)*tspan;
            ry1 = ry + vgrad(2)*tspan;
            j = grid.coord2index([rx1,ry1]);
            thetaN1 = atan2(gradY(j(1),j(2)),gradX(j(1),j(2)));
            
            thetaDdiff = atan2(sin(thetaN1-thetaN),cos(thetaN1-thetaN));
            thetaDdot = thetaDdiff/tspan;
            Kc = 10; eps = 0.0001; v = 1;
            Kw = (thetaDdot + Kc * abs(thetaDiff)^v * ...
                sign(thetaDiff))/(thetaDiff+eps);
            wr = (abs(thetaDiff) >= eps) * Kw * (thetaDiff);
        end
    end
end
