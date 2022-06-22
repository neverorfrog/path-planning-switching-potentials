classdef SwitchingRobot < Robot
    
    properties
        act = Act();
        sense = Sense();
        plan = SwitchingPlan();
    end
        
    methods
        function obj = SwitchingRobot(R,L,grid)
            obj@Robot(R,L,grid);
            obj.plan.inizializza(grid);
        end
        
        function pngSequence(~,samples)
            filename = sprintf('SwitchingPotentials/Latex/presentazione/figure/simulazione/pic%d.png', samples);
            saveas(gcf, filename);
        end
    end
end
