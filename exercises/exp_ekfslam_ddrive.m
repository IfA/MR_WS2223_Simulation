% Perform Localization & SLAM experiments with a single or multiple robots
% Inputs:
%  - filter algorithm: available localization algorithms: 'EKF', 'EIF', 'UKF', 'PF'|'MCL'
%                      available SLAM algorithms: 'EKFSLAM'|'SLAM'
%                      default: 'EKF'
%  - number of robots: default = 1

function exp = exp_ekfslam_ddrive(varargin) 
    exp = experiment_base('ekfslam_ddrive');
    
    % Some configuration options for the experiment
    useEmptyRoom = false;
    useFilterPose = true;
        
    if useEmptyRoom
        % Prepare map and (rectangular) path for empty-room experiment setup
        exp.environment = grp_obstacles_and_landmarks_from_image('../maps/emptyroom.png', 'scale', 0.01);
        pathPoints = [2.00, 0.75; ...
                      2.00, 4.75; ...
                      6.00, 4.75; ...
                      6.00, 0.75; ...
                      2.00, 0.75];
    else
        % Prepare map and path for office environment experiment setup
        exp.environment = grp_obstacles_and_landmarks_from_image('../maps/office.png', 'scale', 0.01);    
        pathPoints = [1.00,   0.50;...
                      1.00, 4.30;...
                      2.60, 4.30;...
                      2.60, 2.70;...
                      6.00, 2.70;... 
                      6.00, 5.10;...
                      7.00, 5.10;...
                      6.00, 5.10;...
                      6.00, 3.30;...
                      7.50, 2.70;...
                      7.50, 0.75;...
                      5.00, 1.50;...
                      2.00, 1.50;...
                      2.00, 0.50];
    end
    
    % The first path point is used as initial pose
    exp.robot.initialPose = [pathPoints(1, :), 90 * pi / 180];
    
    % instantiate the platform (differential drive) and set its parameters
    exp.robot.platform = model_platform2d_ddrive();
    exp.robot.radius = 0.14;
    exp.robot.color = [0 0 1];
    exp.robot.wheelRadius = [0.03 0.03];
    exp.robot.wheelDistance = 0.25;    
        
    % add the controller (which uses the 'platform' as input by default)
    exp.robot.controller = controller_ddrive_follow_path(pathPoints);
    if useFilterPose
        exp.robot.localizationPose = block_extract('slam', 'pose');
        exp.robot.controller.depends = {'localizationPose'};
    end    
    
    % add the sensors
    exp.robot.sensors.odometer = sensor_odometer_wheelspeed();
    exp.robot.odometryError = 10 * pi / 180;        
    
    exp.robot.sensors.landmark_detector = sensor_landmarks2d();
    exp.robot.sensors.range = 7;
    exp.robot.sensors.fieldOfView = 70 * pi / 180 * [-1 1];
    exp.robot.bearingError = 5 * pi / 180;
    exp.robot.rangeError = 2 / 100;    
    
    % ...and finally the localization filter
    % (variant 1: prediction using ODE45; linearize, then discretize; use
    % cartesian measurements)
    exp.robot.slam = filter_ddrive_ekfslam();   
    exp.robot.slam.useNumericPrediction = false;
    exp.robot.slam.useExactDiscretization = true;
    % (This is only required when useFilterPose = false)
	exp.depends = {'*slam'};

    
    exp.display.title = 'SLAM with Extended Kalman Filter';    
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};                            
end