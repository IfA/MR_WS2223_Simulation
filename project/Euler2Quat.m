function [ q ] =Euler2Quat(Orientation)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% M-file: Tool_RPY2Quat
%         -> impelmeted as M-Function
% Author: Marcel Tkocz
% Version: 27.01.2008
%
% Description:
%   This Function converts roll-pitch-yaw angels into their quaternion
%   equivalents.
%
% Inputs:
%   Orientation     ... [vector] orientation in roll-pitch-yaw angels
%
% Outputs:
%   q               ... [vector] orientation in quaternions
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--------------------------------------------------------------------------

a = Orientation(1);        % ... Roll
b = Orientation(2);        % ... Pitch
c = Orientation(3);        % ... Yaw

%--------------------------------------------------------------------------
% Calculation of Direction Cosine Matrix from Roll-Pitch-Yaw angels
%--------------------------------------------------------------------------

R = [ cos(c)*cos(b), cos(c)*sin(b)*sin(a) - sin(c)*cos(a), cos(c)*sin(b)*cos(a) + sin(c)*sin(a);
      sin(c)*cos(b), sin(c)*sin(b)*sin(a) + cos(c)*cos(a), sin(c)*sin(b)*cos(a) - cos(c)*sin(a);
     -sin(b),        cos(b)*sin(a),                        cos(b)*cos(a)                       ];

%--------------------------------------------------------------------------
% Calculation of Quaternions based on q1
%--------------------------------------------------------------------------

q1 = 1/2*sqrt(1 + R(1,1) - R(2,2) - R(3,3));
q2 = 1/(4*q1)*( R(1,2) + R(2,1));
q3 = 1/(4*q1)*( R(1,3) + R(3,1));
q4 = 1/(4*q1)*(-R(2,3) + R(3,2));

if round(q1) ~= 0
    q = [q1; q2; q3; q4];
end

%--------------------------------------------------------------------------
% Calculation of Quaternions based on q2
%--------------------------------------------------------------------------

q2 = 1/2*sqrt(1 - R(1,1) + R(2,2) - R(3,3));
q1 = 1/(4*q2)*( R(1,2) + R(2,1));
q3 = 1/(4*q2)*( R(2,3) + R(3,2));
q4 = 1/(4*q2)*( R(1,3) - R(3,1));

if round(q2) ~= 0
    q = [q1; q2; q3; q4];
end

%--------------------------------------------------------------------------
% Calculation of Quaternions based on q3
%--------------------------------------------------------------------------

q3 = 1/2*sqrt(1 - R(1,1) - R(2,2) + R(3,3));
q1 = 1/(4*q3)*( R(1,3) + R(3,1));
q2 = 1/(4*q3)*( R(2,3) + R(3,2));
q4 = 1/(4*q3)*(-R(1,2) + R(2,1));

if round(q3) ~= 0
    q = [q1; q2; q3; q4];
end

%--------------------------------------------------------------------------
% Calculation of Quaternions based on q4
%--------------------------------------------------------------------------

q4 = 1/2*sqrt(1 + R(1,1) + R(2,2) + R(3,3));
q1 = 1/(4*q4)*(-R(2,3) + R(3,2));
q2 = 1/(4*q4)*(-R(3,1) + R(1,3));
q3 = 1/(4*q4)*(-R(1,2) + R(2,1));
    
if round(q4) ~= 0
    q = [q1; q2; q3; q4];
end

end
