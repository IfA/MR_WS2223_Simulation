% Parameters
% WGS_pos - GPS measurement (in WGS84 format)
% NED_sigma - GPS uncertainty information expressed in {N}
function [WE] = System_W (WGS_pos, NED_sigma)    
    % TODO: implement the measurement covariance matrix
    % Note: In general, this is not a diagonal matrix!
    
    WE = eye(3);

end