classdef Sense
    properties
        
    end
        grid; 
    methods
        function obj = Sense(inputArg1,inputArg2)
            %VISION Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
       function [detections,distances] = scan(r,obstacles,G,n)
            detections = zeros(n,1); distances = zeros(n,1);
            for j = 1 : n
                o = obstacles(j);
                if o.bypassed == false
                    distances(j) = inf;
                else
                    distances(j) = norm([r.xc r.yc] - [o.xc o.yc]);
                end
                detections(j) = (distances(j) <= obj.rv || isinf(distances(j))) && tube(obj,G,o);
            end
        end
    end
end

