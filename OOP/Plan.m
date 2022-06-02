classdef Plan
    
    properties
        agradX; agradY; % the attractive potential every time
        gradX; gradY; %the bypassing potential at time t
        gradXO; gradYO; %the bypassing potential of the detected obstacle at time tau
        P1;
        P2; solxp2; solyp2;
        state;
        
        robot; grid;
    end
    
    methods
        %%
        function obj = Plan(robot,grid)
            obj.robot = robot;
            obj.grid = grid;
            obj.state = State.attractive;
            di = sqrt((grid.goal(1)-grid.X).^2 + (grid.goal(2)-grid.Y).^2);
            obj.agradX = (grid.goal(1)-grid.X)./di; obj.agradY = (grid.goal(2)-grid.Y)./di;
            obj.gradX = obj.agradX; obj.gradY = obj.agradY;
            [obj.solxp2 , obj.solyp2] = calcoloP2();
        end
        
        %% Per ora solo uno alla volta
        function obj = decide(obj,dObstacle)
            rx = obj.robot.xc; ry = obj.robot.yc;
            if obj.state == State.attractive
                if isempty(dObstacle) %obstacle not detected
                    return;
                end
                obj = obj.bypass(dObstacle);
                obj.state = State.bypassing;
            else
                if norm([rx ry] - obj.P1) < 0.1
                    obj = obj.setGrad(obj.gradXO,obj.gradYO); return;
                end
                if norm([rx ry] - obj.P2) < 0.1
                    obj = obj.setGrad(obj.agradX,obj.agradY);
                    obj.state = State.attractive; return;
                end
            end
        end
        
        function obj = setGrad(obj,gradX,gradY)
            obj.gradX = gradX; obj.gradY = gradY;
        end
        
        %% Metodo che calcola il ptoenziale bypassante
        function obj = bypass(obj,dO)
            %Posizione del robot,dell'ostacolo e del goal
            xr = obj.robot.xc; yr = obj.robot.yc; thetar = obj.robot.theta;
            xo0 = dO.xc; yo0 = dO.yc; thetao0 = dO.theta;
            angle = atan2(sin(thetar),cos(thetar));
            xg = obj.grid.goal(1); yg = obj.grid.goal(2);
            
            %Inizio calcolo potenziale bypassante
            m = tan(obj.robot.theta); xo = xo0 - xr; yo = yo0 - yr; 
            %1. Calcolo di h (il raggio della circonferenza attorno all'ostacolo reale)
            dist = norm([xo0 yo0] - [xr yr]);
            angdiff = abs(atan2(sin(thetao0-thetar),cos(thetao0-thetar)));
            h = (dist+1+(angdiff/pi))/3;
            %2. Calcolo delle due circonferenze papabili per l'ostacolo virtuale
            yOmega(1) = ((- h^2 + xo^2 + yo^2)*(m*xo - yo + h*(m^2 + 1)^(1/2)))/(2*h^2*m^2 + 2*h^2 - 2*m^2*xo^2 + 4*m*xo*yo - 2*yo^2);
            yOmega(2) = -((-h^2 + xo^2 + yo^2)*(yo - m*xo + h*(m^2 + 1)^(1/2)))/(2*h^2*m^2 + 2*h^2 - 2*m^2*xo^2 + 4*m*xo*yo - 2*yo^2);
            xOmega = -m*yOmega;
            d = double(sqrt(xOmega.^2 + yOmega.^2));
            %3. Scelta del verso di bypass dell'ostacolo reale
            oSense = obj.chooseSense(dO); %dO = detected obstacle
            %4. Decido quale circonferenza va bene per il verso di bypass
            if (abs(norm(xOmega)) > 0.01 && sign(xOmega(1)) == sign(xOmega(2))) ... %due circonferenze dallo stesso lato
                    || (abs(norm(yOmega)) > 0.01 && sign(yOmega(1)) == sign(yOmega(2)))
                [~,indiceC] = min(d);
            else
                if oSense == "clock" %voglio la circonferenza sinistra
                    indiceC = (sign(sin(angle)) == sign(xOmega(1))) + 1;
                else %voglio quella destra
                    indiceC = ~(sign(sin(angle)) == sign(xOmega(1))) + 1;
                end
            end
            %5. Estraggo la circonferenza che mi serve
            xOmega = xOmega(indiceC) + xr; yOmega = yOmega(indiceC) + yr; 
            dOmega = d(indiceC); xo = xo0; yo = yo0;
            %6. Decido come sará l'ostacolo virtuale in base al senso di bypass
            vObstacle = Obstacle(xOmega,yOmega,[0;0]);
            if oSense == "counterclock"
                vSense = "clock"; indiceP2 = 2;
            else
                vSense = "counterclock"; indiceP2 = 1;
            end
            %7. Calcolo P1
            centerDir = [xo,yo]-[xOmega,yOmega];
            centerDir = centerDir/norm(centerDir);
            obj.P1 = [xOmega yOmega] + dOmega*centerDir;
            k = obj.grid.coord2index(obj.P1);
            %8. Calcolo P2
            obj.P2 = double(subs(obj.solxp2(indiceP2)));
            syms xp2; xp2 = obj.P2(1); %#ok<*NASGU>
            obj.P2(2) = double(subs(obj.solyp2(indiceP2)));
            j = obj.grid.coord2index(obj.P2);
            %9. Infine calcolo gli antigradienti nell'ostacolo reale e virtuale
            cO = norm([obj.gradX(j(1),j(2)) obj.gradY(j(1),j(2))])*h;
            [obj.gradXO,obj.gradYO] = obj.antigradient(dO,obj.grid,cO,oSense);
            cV = norm([obj.gradXO(k(1),k(2)) obj.gradYO(k(1),k(2))])*dOmega;
            [obj.gradX,obj.gradY] = obj.antigradient(vObstacle,obj.grid,cV,vSense);
            
             %Plotting dei risultati per testing
            omega = nsidedpoly(2000, 'Center', [double(xOmega) double(yOmega)], 'Radius', double(dOmega));
            plot(omega, 'FaceColor', 'b');
            obst = nsidedpoly(2000, 'Center', [xo yo], 'Radius', double(h));
            plot(obst, 'FaceColor', 'r');
            plot(obj.P2(1),obj.P2(2),"+k","linewidth",2); plot(obj.P1(1),obj.P1(2),"+b","linewidth",2);
        end
        
        %% Scelto del senso (antiorario o orario)
        function sense = chooseSense(obj,obstacle)
            phi = atan2(obstacle.yc - obj.robot.yc,obstacle.xc - obj.robot.xc);
            alphav = atan2(obstacle.v(2),obstacle.v(1));
            vphi = atan2(sin(-phi + pi/2 + alphav),cos(-phi + pi/2 + alphav));
            if obstacle.v(1) == 0 && obstacle.v(2) == 0
                if phi - obj.robot.theta > 0
                    sense = "counterclock";
                else
                    sense = "clock";
                end
            else
                if cos(vphi) > 0
                    sense = "clock";
                else
                    sense = "counterclock";
                end
            end
        end
        
        %% Calcolo antigradiente dato c e il senso
        function [gx,gy] = antigradient(~,obstacle,grid,c,sense)
            xo = obstacle.xc; yo = obstacle.yc;
            %Calcolo del gradiente
            X = grid.X; Y = grid.Y;
            gx = c*(Y(:,1)-yo)./((X(1,:)-xo).^2+(Y(:,1)-yo).^2+0.00001);
            gy = c*(xo-X(1,:))./((X(1,:)-xo).^2+(Y(:,1)-yo).^2+0.00001);
            if sense == "counterclock"
                gx = -gx; gy = -gy;
            end
        end  
    end
    
end
