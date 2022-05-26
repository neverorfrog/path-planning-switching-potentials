classdef Grid < handle
    
    properties (SetAccess = immutable)
        ordine; fattore;
        nr; nc;
        X; Y;
    end
    
    properties 
        goal; obstacles;
    end
    
    methods
        function obj = Grid(ordine,x0,xf)
            obj.ordine = ordine;
            obj.fattore = (10^ordine);
            obj.nr = obj.fattore*10 + 1;
            obj.nc = obj.fattore*10 + 1;
            [obj.X,obj.Y] = meshgrid(linspace(x0,xf,obj.nc),linspace(x0,xf,obj.nr));
        end
        
        function obj = setGoal(obj,goal)
            obj.goal = goal;
        end
        
        function obj = addObstacle(obj,o)
            obj.obstacles = [obj.obstacles , o];
        end
        
        function index = coord2index(obj,point)
            index = zeros(1,2);
            index(1) = int16(round(point(1),obj.ordine)*obj.fattore+1);
            index(2) = int16(round(point(2),obj.ordine)*obj.fattore+1);
        end
    end
end

