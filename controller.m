classdef controller
    %PI controller for the robot steering
    %   priorValues is used to implement static memory
    
    properties
        priorValues
        numValues
    end
    
    methods
        function obj = controller()
            obj.numValues = 0;
        end
        function output_angle = steer(obj,input_angle)
            
            if (obj.numValues < 5)
                obj.numValues = obj.numValues + 1;
                obj.priorValues(obj.numValues) = input_angle;
            else
                obj.priorValues = circshift(obj.priorValues,-1);
                obj.priorValues(5,1) = input_angle;
            end
            
            output_angle = 0.5*input_angle + 0.5*(mean(obj.priorValues))
        end
    end
    
end

