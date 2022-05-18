classdef Grid < handle
    
    properties (SetAccess = immutable)
        ordine; fattore;
        nr; nc;
        X; Y;
    end
    
    properties
        G;
        agradX; agradY;
    end
    
    methods
        function obj = Grid(ordine,x0,xf)
        obj.ordine = ordine;
        obj.fattore = (10^ordine);
        obj.nr = obj.fattore*10 + 1; 
        obj.nc = obj.fattore*10 + 1; 
        [obj.X,obj.Y] = meshgrid(linspace(x0,xf,obj.nc),linspace(x0,xf,obj.nr));
        end
        
        function obj = setGoal(obj,G)
            obj.G = G;
            di = sqrt((G(1)-obj.X).^2 + (G(2)-obj.Y).^2);
            obj.agradX = (G(1)-obj.X)./di; obj.agradY = (G(2)-obj.Y)./di;
        end
        
        function obj = setGrad(gradX,gradY)
            obj.agradX = gradX;  obj.agradY = gradY;
        end
        
        function index = coord2index(obj,point)
            index(1) = int16(round(point(1),obj.ordine)*obj.fattore+1);
            index(2) = int16(round(point(2),obj.ordine)*obj.fattore+1);
        end
    end
end

