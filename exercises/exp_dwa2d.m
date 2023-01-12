function exp = exp_dwa2d() 
    exp = experiment_base('dwa2d');
        
    % select the scenario
    scenario = 1;
    
    % Prepare list of waypoints for office environment experiment setup
    switch scenario
        case 1
            pathPoints = [1.00,   0.50;...
                          1.00, 4.10;...
                          2.60, 4.30;...
                          2.60, 2.70;...
                          5.50, 2.70;... 
                          6.00, 5.10;...
                          7.00, 5.10;...
                          6.00, 5.10;...
                          6.00, 3.30;...
                          7.50, 2.70;...
                          7.50, 0.75;...
                          5.00, 1.50;...
                          2.00, 1.50;...
                          2.00, 0.50];
            exp.robot.initialPose = [pathPoints(1, :), 90 * pi / 180];    
		case 2
			pathPoints = [2.00, 1.14; ...
						  5.00, 1.14];	
            exp.robot.initialPose = [pathPoints(1, :), 0 * pi / 180];                          
		otherwise
            error('Unknown scenario selected');
    end
    
    exp.robot.path = const_points(pathPoints);
    exp.robot.path.format = {'Color', [0 0 1], 'Marker', 'x'};

    % load the environment map
    [exp.environment] = grp_obstacles_and_landmarks_from_image('../maps/office.png', ...
                                                               'scale', 0.01);        

    % instantiate the platform (v/omega drive) and set its parameters
    exp.robot.platform = model_platform2d_vomega();
    exp.robot.radius = 0.14;
    exp.robot.color = [0 0 1];
        
    % add the sensors
    exp.robot.sensors.rangefinder = sensor_rangefinder2d();
    exp.robot.sensors.rangefinder.maxRange = 3;                                                                   
    exp.robot.sensors.rangefinder.fieldOfView = [-90, 90] * pi / 180;
    exp.robot.sensors.rangefinder.color = [0.7 0 0];
    exp.robot.sensors.rangefinder.timing.deltaT = 1 / 10; % Sample time of the laser rangefinder (propagates to the DWA module via the trigger mechanism)
    
    % the robot is controlled by the Dynamic Window Approach
    exp.robot.controller = guidance_dwa2d();
    
    % Another control 'layer' provides intermediate goals for the DWA to
    % prevent it from getting stuck while travelling through the office
    % environment    
    exp.robot.targetProvider = guidance_waypoints();        
    exp.robot.targetProvider.relative = true;
    exp.robot.controller.depends{2} = 'targetProvider';    
    
    % Some main Window display settings
    exp.display.title = 'Dynamic Window Approach (DWA)';        
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};
    
end