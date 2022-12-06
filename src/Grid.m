classdef Grid < handle
    
    properties (SetAccess = immutable)
        resolution; %densitá (quante celle per quadratino di coordinata)
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
        function obj = Grid(resolution,width)
            obj.resolution = resolution;
            obj.dx = 1/resolution;
            obj.nr = width*resolution + 1;
            obj.nc = obj.nr;
            [obj.X,obj.Y] = meshgrid(linspace(0,width,obj.nc),linspace(0,width,obj.nr));
            obj.x = linspace(0,width,obj.nc);
            obj.y = linspace(0,width,obj.nr)';
            obj.width = width;
        end
        
        function obj = setGoal(obj,goal)
            obj.goal = goal;
        end
        
        function obj = addObstacle(obj,o)
            obj.obstacles = [obj.obstacles o];
        end
        
        function obj = moveObstacles(obj,tspan)
            for k = 1 : length(obj.obstacles)
                obj.obstacles(k).move(tspan);
            end
        end
        
        function index = coord2index(obj,point)
            index = zeros(1,2);
            % x coordinate goes into the column index
            floorx = floor(point(1));
            index(2) = (floorx + obj.dx*floor((point(1) - floorx)/obj.dx))*obj.resolution + 1;
            % y coordinate goes into the row index
            floory = floor(point(2));
            index(1) = (floory + obj.dx*floor((point(2) - floory)/obj.dx))*obj.resolution + 1;
            index = floor(index);
        end
    end
end

