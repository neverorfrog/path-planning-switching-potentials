classdef Map < occupancyMap
    
    methods
        function obj = Map(width,height,resolution)
            obj@occupancyMap(width,height,resolution);
        end
    end
end
