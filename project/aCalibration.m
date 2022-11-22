function [sigma_out, mean_out, bias_out] = aCalibration (a)
    
    sigma_out = std(a, 0, 2);
    mean_out = mean(a, 2);
    
    % The direction of the mean acceleraion is assumed to be the direction 
    % of gravity (thus down). Any other component is considered bias. This 
    % is by no means an acurate representation of the biases. 
    bias_out = mean_out - mean_out / norm(mean_out) * 9.81;
    
end