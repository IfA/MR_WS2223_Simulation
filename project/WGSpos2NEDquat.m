function NEDquat = WGSpos2NEDquat(WGSpos)

    % TODO: Compute the Attitude of an NED frame of which the origin is
    % located at the given WGS84 position
    R = eye(3);
    
    NEDquat = DCM2Quat(R);

end

