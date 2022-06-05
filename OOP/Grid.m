classdef Grid < handle
    
    properties (SetAccess = immutable)
        fattore; %densitá (quante celle per quadratino di coordinata)
        nr; nc;
        dx;
        X; Y;
        x; y;
        width;
    end
    
    properties 
        goal; 
        obstacles;
    end
    
    methods
        function obj = Grid(fattore,x0,xf)
            obj.fattore = fattore;
            obj.dx = 1/fattore;
            obj.nr = (xf - x0)*fattore + 1;
            obj.nc = obj.nr;
            [obj.X,obj.Y] = meshgrid(linspace(x0,xf,obj.nc),linspace(x0,xf,obj.nr));
            obj.x = linspace(x0,xf,obj.nc);
            obj.y = linspace(x0,xf,obj.nr)';
            obj.width = xf - x0;
        end
        
        function obj = setGoal(obj,goal)
            obj.goal = goal;
        end
        
        function obj = addObstacle(obj,o)
            obj.obstacles = [obj.obstacles o];
        end
        
        function index = coord2index(obj,point)
            index = zeros(1,2);
            % x coordinate goes into the column index
            floorx = floor(point(1));
            index(2) = (floorx + obj.dx*floor((point(1) - floorx)/obj.dx))*obj.fattore + 1;
            % x coordinate goes into the row index
            floory = floor(point(2));
            index(1) = (floory + obj.dx*floor((point(2) - floory)/obj.dx))*obj.fattore + 1;
            index = floor(index);
        end
        
        function checkOccupancy
    end
end

