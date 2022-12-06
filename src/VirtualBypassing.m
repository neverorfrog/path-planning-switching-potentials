classdef VirtualBypassing < Bypassing
    
    properties
        gradXO; gradYO; %the bypassing potential of the detected obstacle at time tau
        P1; %punto in cui si passa dal bypassante virtuale a quello reale
        P2; %punto in cui si passa dal bypassante all'attrattivo
    end
    
    methods
        function obj = decision(obj,robot,dObstacle)
            decision@Bypassing(obj,robot,dObstacle);
            if norm([robot.xc robot.yc] - obj.P1) < 0.1
                robot.setState(RealBypassing(obj));
            end
        end
        
        function [obj,success] = VirtualBypassing(robot,dO)
            %Posizione del robot,dell'ostacolo e del goal
            pose = robot.getPose(); grid = robot.grid;
            xr = pose(1); yr = pose(2); thetar = pose(3);
            xo0 = dO.xc; yo0 = dO.yc; thetao0 = dO.theta;
            angle = atan2(sin(thetar),cos(thetar));
            xg = grid.goal(1); yg = grid.goal(2);
            m = tan(thetar); xo = xo0 - xr; yo = yo0 - yr;
            
            %Inizio calcolo potenziale bypassante
            %1. Calcolo del verso di bypassing e di h
            [oSense,h] = obj.chooseSense(dO,robot); %dO = detected obstacle
            %2. Calcolo delle due circonferenze papabili per l'ostacolo virtuale
            load yOmega; yOmega = double(subs(yOmega(:)));
            xOmega = -m*yOmega;
            d = double(sqrt(xOmega.^2 + yOmega.^2));
            %3. Decido quale circonferenza va bene per il verso di bypass
            boolclock = oSense == "clock";
            signX = sign(xOmega); signY = sign(yOmega);
            bool1 = abs(norm(xOmega))>0.01 && signX(1) == signX(2);
            bool2 = abs(norm(yOmega))>0.01 && signY(1)==signY(2);
            if bool1 || bool2 %Ho due circonferenze dallo stesso lato
                if boolclock && bool1 && sign(sin(angle)) == signX(1) || ...
                        ~boolclock && bool1 && sign(sin(angle)) ~= signX(1) || ...
                        boolclock && sign(sin(angle))==0 && signY(1) == -1 || ...
                        ~boolclock && sign(sin(angle))==0 && signY(1) == 1
                    success = 0;
                    return;
                else
                    [~,indiceC] = min(d);
                end
            else %Non ho due circonferenze dallo stesso lato
                if oSense == "clock" %voglio la circonferenza sinistra
                    indiceC = (sign(sin(angle)) == sign(xOmega(1))) + 1;
                else %voglio quella destra
                    indiceC = ~(sign(sin(angle)) == sign(xOmega(1))) + 1;
                end
            end
            xOmega = xOmega(indiceC) + xr; yOmega = yOmega(indiceC) + yr;
            dOmega = d(indiceC); xo = xo0; yo = yo0;
            %4. Decido come sara l'ostacolo virtuale in base al senso di bypass
            vObstacle = Obstacle(xOmega,yOmega,[0;0]);
            if oSense == "counterclock"
                vSense = "clock"; indiceP2 = 2;
            else
                vSense = "counterclock"; indiceP2 = 1;
            end
            %5. Calcolo P1
            centerDir = [xo,yo]-[xOmega,yOmega];
            centerDir = centerDir/norm(centerDir);
            obj.P1 = [xOmega yOmega] + dOmega*centerDir;
            %6. Calcolo P2
            load xp2yp2;
            obj.P2 = double(subs(solxp2(indiceP2)));
            syms xp2; xp2 = obj.P2(1);
            obj.P2(2) = double(subs(solyp2(indiceP2)));
            %7a. Infine calcolo l'antigradiente nell'ostacolo reale
            agradX = robot.attractive.gradX; agradY = robot.attractive.gradY;
            j = grid.coord2index(obj.P2);
            cO = norm([agradX(j(1),j(2)) agradY(j(1),j(2))])*h*ones(length(agradY));
            [obj.gradXO,obj.gradYO] = obj.antigradient(dO,grid,cO,oSense);
            %7b. E in quello virtuale
            dr = sqrt((pose(1)-robot.grid.X).^2 + (pose(2)-robot.grid.Y).^2);
            dp = sqrt((obj.P1(1)-robot.grid.X).^2 + (obj.P1(2)-robot.grid.Y).^2);
            drp = sqrt((obj.P1(1)-pose(1))^2 + (obj.P1(2)-pose(2))^2);
            k = grid.coord2index(obj.P1);
            i = grid.coord2index([pose(1) pose(2)]);
            currentgradX = robot.state.gradX;
            currentgradY = robot.state.gradY;
            cr = norm([currentgradX(i(1),i(2)) currentgradY(i(1),i(2))])*dOmega;
            cp = norm([obj.gradXO(k(1),k(2)) obj.gradYO(k(1),k(2))])*dOmega;
            cV = cr*(dp/drp) + cp*(dr/drp); %- (dr.*dp)/4;
            [obj.gradX,obj.gradY] = obj.antigradient(vObstacle,grid,cV,vSense);
            success = 1;
            
            %Plotting dei risultati per testing
            persistent plotobj1; persistent plotobj2; persistent plotobj3; persistent plotobj4;
            delete(plotobj1); delete(plotobj2); delete(plotobj3); delete(plotobj4);
            omega = nsidedpoly(2000, 'Center', [xOmega yOmega], 'Radius', dOmega);
            obst = nsidedpoly(2000, 'Center', [xo yo], 'Radius', double(h));
            plotobj1 = plot(omega, 'FaceColor', 'b');
            plotobj2 = plot(obst, 'FaceColor', 'm');
            plotobj3 = plot(obj.P2(1),obj.P2(2),"+k","linewidth",2);
            plotobj4 = plot(obj.P1(1),obj.P1(2),"+b","linewidth",2);
        end
        
        %% Scelto del senso (antiorario o orario)
        function [sense,h] = chooseSense(~,obstacle,robot)
            pose = robot.getPose(); xr = pose(1); yr = pose(2); thetar = pose(3);
            phi = atan2(obstacle.yc - yr,obstacle.xc - xr);
            %Scelta verso
            Rphi = [cos(phi-pi/2) sin(phi-pi/2) ; cos(phi) sin(phi)];
            vphi = Rphi*obstacle.v;
            thetaphi = Rphi*[cos(thetar),sin(thetar)]';
            %È fermo
            if obstacle.v(1) == 0 && obstacle.v(2) == 0
                if cos(atan2(thetaphi(2),thetaphi(1))) > 0
                    sense = "counterclock";
                else
                    sense = "clock";
                end
                %Non é fermo
            else
                if cos(atan2(vphi(2),vphi(1))) > 0
                    sense = "clock";
                else
                    sense = "counterclock";
                end
            end
            
            %Calcolo h
%             thetar = atan2(robot.grid.goal(2) - yr, robot.grid.goal(1) - xr);
            Rtheta = [cos(thetar-pi/2) sin(thetar-pi/2) ; cos(thetar) sin(thetar)];
            vtheta= Rtheta*obstacle.v;
            alphavtheta = atan2(vtheta(2),vtheta(1));
            angdiff = abs(atan2(sin(-pi/2-alphavtheta),cos(-pi/2-alphavtheta)));
            rv = robot.rv; ro = obstacle.raggio;
            minlim = 0.2; maxlim = 0.2;
            hmin = ro+minlim; hmax = rv - maxlim;
            gradoinv = abs(1-angdiff/(pi/2));
            h = ((hmax-hmin)*gradoinv + hmin);
        end
        
        %% Calcolo antigradiente dato c e il verso di percorrenza
        function [gx,gy] = antigradient(~,obstacle,grid,c,sense)
            xo = obstacle.xc; yo = obstacle.yc;
            %Calcolo del gradiente
            x = grid.x; y = grid.y;
            gx = c.*(y-yo)./((x-xo).^2+(y-yo).^2+0.00001);
            gy = c.*(xo-x)./((x-xo).^2+(y-yo).^2+0.00001);
            if sense == "counterclock"
                gx = -gx; gy = -gy;
            end
        end
    end
end
