function [output_angle, new_priorValues] = steer(input_angle,priorValues)
%PI Controller for the robot's steering angle
%   priorValues is a variable kept in the calling scope
%   priorValues is a column vector of the past 5 input headings

    new_priorValues = circshift(priorValues,-1);
    new_priorValues(5,1) = input_angle;

    %Adjusts these coefficients to tune PI performance
    output_angle = 0.5*input_angle + 0.5*(mean(priorValues));
end

