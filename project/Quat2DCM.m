function [ R ] = Quat2DCM( q )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% M-file: Tool_Quat2DCM
%         -> impelmeted as M-Function
% Author: Marcel Tkocz
% Version: 23.01.2008
%
% Description:
%   Calculation of the direction cosine matrix for an input
%   orientation
%
% Inputs:
%   q               ... [vector] orientation in quaternions
%
% Outputs:
%   R               ... [matrix] direction cosine matrix for the orientation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--------------------------------------------------------------------------
% Calculation of direction cosine matrix
%--------------------------------------------------------------------------

R = [q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2,  2*(q(1)*q(2) - q(3)*q(4)),          2*(q(1)*q(3) + q(2)*q(4));
     2*(q(1)*q(2) + q(3)*q(4)),         -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2,  2*(q(2)*q(3) - q(1)*q(4));
     2*(q(1)*q(3) - q(2)*q(4)),          2*(q(2)*q(3) + q(1)*q(4)),         -q(1)^2 - q(2)^2 + q(3)^2 + q(4)^2];

end