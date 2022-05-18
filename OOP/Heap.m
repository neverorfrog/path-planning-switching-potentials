classdef Heap < Handle

    properties (Access = private)
        elem;
        pos;
    end
    
    methods
        function obj = Heap()
            obj.pos = 1;
        end    
        
        function result = top(obj)
            if obj.pos == 1 
                result = -1; return;
            end
            result = obj.elem(1);
        end
        
        function in(obj,x)
            obj.elem(obj.pos) = x; 
            obj.pos = obj.pos + 1;
            tmpPos = obj.pos - 1;
            while tmpPos > 1
                if obj.elem(tmpPos) > obj.elem(padre(tmpPos))
                    break;
                end
                tmpElem = obj.elem(tmpPos);
                
            end
        end
    end
    
    methods (Access = private)
        function result = padre(tmpPos)
            result = tmpPos/2;
        end
        
        function result = figlioSin(tmpPos)
            result = tmpPos*2;
        end
        
        function result = figlioDes(tmpPos)
            result = tmpPos*2+1;
        end
    end
end

