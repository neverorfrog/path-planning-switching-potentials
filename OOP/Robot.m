classdef Robot < matlab.mixin.Copyable
    
    %Object that represents a robot of triangular shape in position (xc,yc) and a certain orientation
    %theta wrt the positive x axis in R2
    
    properties %position
        xc; yc; theta; %position and orientation
        rv; %vision radius
    end
    
    properties %plotting
        plotobj; %graphic handle
        shape; %circular or triangular
    end
    
    properties %bypassing potential
        obstacle; %the obstacle to be bypassed
        state; %can be bypassing or attractive
        gradX; gradY; %the antigradient to be followed
        gradXO; gradYO; %the bypassing potential of the detected obstacle
        P1; P2; solxp2; solyp2;
    end
    
    
    methods
        %% Constructor
        function obj = Robot(xc,yc)
            obj.xc = xc; obj.yc = yc; obj.theta = pi/2;
            obj.shape = "triangular";
            obj.rv = 1.5;
            obj.obstacle = Obstacle(inf,inf,[0 0]);
            obj.state = State.attractive;
            [obj.solxp2 , obj.solyp2] = calcoloP2();
        end
        
        
        %% Graphic representation of the robot
        function plotobj = draw(obj)
            if obj.shape == "triangular"
                o = pi/2 - atan2(sin(obj.theta),cos(obj.theta)); %orientation
                p = [obj.xc;obj.yc]; %position
                c = [cos(o) sin(o) ; -sin(o) cos(o)] * [0 1 -1 ; 1 -1 -1] * 0.3 + p; %coordinates
                delete(obj.plotobj);
                plotobj = plot(polyshape(c(1,:),c(2,:)),"LineWidth",2,"FaceColor","b");
                hold on; axis equal; axis([-1 11 -1 11]); plot(obj.xc,obj.yc,"ob");
                obj.plotobj = plotobj;
            elseif obj.shape == "circular"
                r = 0.3; x = obj.xc; y = obj.yc; t = 0:0.01:2*pi;
                delete(obj.plotobj);
                plotobj = plot(cos(t)*r+x,sin(t)*r+y,"b","linewidth",2);
                plot(obj.xc,obj.yc,"ob"); hold on; axis equal; axis([-1 11 -1 11]);
                obj.plotobj = plotobj;
            end
        end
        
        %%
        function setGrad(obj,gradX,gradY)
            obj.gradX = gradX; obj.gradY = gradY;
        end
        
        %% Method that moves the robot according to the desired velocity (Runge-Kutta 2 for integration)
        function move(obj,tspan,grid)
            rx = obj.xc;
            ry = obj.yc;
            rtheta = obj.theta;
            [xdot,ydot,thetadot] = obj.derivata(rx,ry,rtheta,tspan,grid);
            
            rx2 = rx + tspan/2*xdot;
            ry2 = ry + tspan/2*ydot;
            rtheta2 = rtheta + tspan/2*thetadot;
            [xdot2,ydot2,thetadot2] = obj.derivata(rx2,ry2,rtheta2,tspan,grid);
            
            obj.xc = rx + tspan*xdot2;
            obj.yc = ry + tspan*ydot2;
            obj.theta = rtheta + tspan*thetadot2;
            
            obj.draw();
        end
        
        
        function [xdot,ydot,thetadot] = derivata(obj,rx,ry,rtheta,tspan,grid)  
            i = grid.coord2index([rx,ry]);
            thetaN = atan2(obj.gradY(i(2),i(1)),obj.gradX(i(2),i(1)));
            thetaDiff = atan2(sin(thetaN-rtheta),cos(thetaN-rtheta));
            vgrad = [obj.gradX(i(2),i(1)) obj.gradY(i(2),i(1))];
            Mv = norm(vgrad);
            vr = (Mv * cos(thetaDiff));
            
            rx1 = rx + vgrad(1)*tspan;
            ry1 = ry + vgrad(2)*tspan;
            j = grid.coord2index([rx1,ry1]);
            thetaN1 = atan2(obj.gradY(j(2),j(1)),obj.gradX(j(2),j(1)));
            
            thetaDdiff = atan2(sin(thetaN1-thetaN),cos(thetaN1-thetaN));
            thetaDdot = thetaDdiff/tspan;
            Kc = 10; eps = 0.001; v = 1;
            Kw = (thetaDdot + Kc * abs(thetaDiff)^v * sign(thetaDiff))/(thetaDiff+eps);
            wr = (abs(thetaDiff) >= eps) * Kw * (thetaDiff);
            
            xdot = vr * cos(rtheta);
            ydot = vr * sin(rtheta);
            thetadot = wr;
        end
        
        %% Method that looks in a radius rv and a tube T(t) if there are any obstacles
        function [detections,distances] = scan(obj,obstacles,G,n)
            detections = zeros(n,1); distances = zeros(n,1);
            for j = 1 : n
                o = obstacles(j);
                if o.bypassed == false
                    distances(j) = inf;
                else
                    distances(j) = norm([obj.xc obj.yc] - [o.xc o.yc]);
                end
                detections(j) = (distances(j) <= obj.rv || isinf(distances(j))) && tube(obj,G,o);
            end
        end
        
        %% Method that builds the tube T(t)
        function result = tube(obj,G,o)
            rm = 3.5;
            angle = atan2(G(2)-obj.yc,G(1)-obj.xc);
            deltaX = rm/2*sin(angle); deltaY = rm/2*cos(angle);
            x1 = obj.xc + deltaX; y1 = obj.yc - deltaY;
            x4 = obj.xc - deltaX; y4 = obj.yc + deltaY;
            x2 = G(1) + deltaX; y2 = G(2) - deltaY;
            x3 = G(1) - deltaX; y3 = G(2) + deltaY;
            
            m14 = (y4-y1)/(x4-x1); q14 = m14*x1 - y1;
            if abs(m14) > exp(10)
                v14 = (o.xc - x1);
            else
                v14 = (o.yc - m14*o.xc + q14);
            end
            
            m12 = (y2-y1)/(x2-x1); q12 = m12*x1 - y1;
            if abs(m12) > exp(10)
                v12 = (x1 - o.xc);
            else
                v12 = (o.yc - m12*o.xc + q12);
            end
            
            m34 = (y4-y3)/(x4-x3); q34 = m34*x3 - y3;
            if abs(m34) > exp(10)
                v34 = (x4 - o.xc);
            else
                v34 = (o.yc - m34*o.xc + q34);
            end
            
            if (angle >= 0 && angle <= pi/2) %first quadrant
                result1 = v14 >= 0;
                result2 = v12 >= 0; result3 = v34 <= 0;
            elseif (angle > pi/2 && angle < pi) %second quadrant
                result1 = v14 >= 0;
                result2 = v12 <= 0; result3 = v34 >= 0;
            elseif (angle >= -pi && angle <= -pi/2 || angle == pi ) %third quadrant
                result1 = v14 <= 0;
                result2 = v12 <= 0; result3 = v34 >= 0;
            else %fourth quadrant
                result1 = v14 <= 0;
                result2 = v12 >= 0; result3 = v34 <= 0;
            end
            result = result1 && result2 && result3;
        end
        
        
        
        
        %% Metodo che calcola il ptoenziale bypassante
        function  bypass(obj,dO,grid)
            
            [vObstacle,h,dOmega] = obj.virtualObstacle(dO,grid.G);
            obj.obstacle = dO; obj.state = State.bypassing; dO.bypassed = false;
            
            j = coord2index(obj.P2);
            k = coord2index(obj.P1);
            
            cO = norm([grid.agradX(j(2),j(1)) grid.agradY(j(2),j(1))])*h;
            [obj.gradXO,obj.gradYO] = dO.antigradient(grid.X,grid.Y,cO,ordine,fattore);
            cV = norm([obj.gradXO(k(2),k(1)) obj.gradYO(k(2),k(1))])*dOmega;
            [obj.gradX,obj.gradY] = vObstacle.antigradient(grid.X,grid.Y,cV,ordine,fattore);
        end
        
        function [vObstacle,h,dOmega] = virtualObstacle(obj,dO,G)
            %calcolo di h (il raggio della circonferenza attorno all'ostacolo reale)
            dist = norm([dO.xc dO.yc] - [obj.xc obj.yc]);
            angdiff = abs(atan2(sin(dO.theta-obj.theta),cos(dO.theta-obj.theta)));
            h = (dist+1+(angdiff/pi))/3;
            angle = atan2(sin(obj.theta),cos(obj.theta));
            
            %substitution of data from detected obstacle to calculate bypass potential
            xo0 = dO.xc; yo0 = dO.yc; xr = obj.xc; yr = obj.yc;
            m = tan(obj.theta); xo = xo0 - xr; yo = yo0 - yr; xg = G(1); yg = G(2);
            
            %calcolo delle due circonferenze
            yOmega(1) = ((- h^2 + xo^2 + yo^2)*(m*xo - yo + h*(m^2 + 1)^(1/2)))/(2*h^2*m^2 + 2*h^2 - 2*m^2*xo^2 + 4*m*xo*yo - 2*yo^2);
            yOmega(2) = -((-h^2 + xo^2 + yo^2)*(yo - m*xo + h*(m^2 + 1)^(1/2)))/(2*h^2*m^2 + 2*h^2 - 2*m^2*xo^2 + 4*m*xo*yo - 2*yo^2);
            xOmega = -m*yOmega;
            d = double(sqrt(xOmega.^2 + yOmega.^2));
            
            %scelta del verso di bypass dell'ostacolo reale
            dO.chooseSense(obj);
            
            %Decido quale circonferenza va bene per il verso di bypass
            if (abs(norm(xOmega)) > 0.01 && sign(xOmega(1)) == sign(xOmega(2))) ... %due circonferenze dallo stesso lato
                    || (abs(norm(yOmega)) > 0.01 && sign(yOmega(1)) == sign(yOmega(2)))
                [~,indiceC] = min(d);
            else
                if dO.sense == "clock" %voglio la circonferenza sinistra
                    indiceC = (sign(sin(angle)) == sign(xOmega(1))) + 1;
                else %voglio quella destra
                    indiceC = ~(sign(sin(angle)) == sign(xOmega(1))) + 1;
                end
            end
            
            xOmega = xOmega(indiceC) + xr; yOmega = yOmega(indiceC) + yr; dOmega = d(indiceC); xo = xo0; yo = yo0;
            vObstacle = Obstacle(xOmega,yOmega,[0;0]);
            if dO.sense == "counterclock"
                vObstacle.sense = "clock"; indiceP2 = 2;
            else
                vObstacle.sense = "counterclock"; indiceP2 = 1;
            end
            
            %calcolo P1
            centerDir = [xo,yo]-[xOmega,yOmega];
            centerDir = centerDir/norm(centerDir);
            obj.P1 = [xOmega yOmega] + dOmega*centerDir;
            
            %calcolo P2
            obj.P2(1) = double(subs(obj.solxp2(indiceP2)));
            syms xp2; xp2 = obj.P2(1);
            obj.P2(2) = double(subs(obj.solyp2(indiceP2)));
            
            omega = nsidedpoly(2000, 'Center', [double(xOmega) double(yOmega)], 'Radius', double(dOmega));
            plot(omega, 'FaceColor', 'b'); hold on;  axis equal;
            obst = nsidedpoly(2000, 'Center', [xo yo], 'Radius', double(h));
            plot(obst, 'FaceColor', 'r'); hold on;
            plot(obj.P2(1),obj.P2(2),"+k","linewidth",2); plot(obj.P1(1),obj.P1(2),"+b","linewidth",2); plot(obj.xc,obj.yc,"+y");
        end
    end
    
end
