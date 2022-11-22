% TUD/IfA, Course "Mobile Robotic" - Practical Project
%
% Process an experiment and visualize the results according to the provided
% option arguments 
%
function project(varargin)
    if nargin == 1
        options = varargin{1};
    else options = struct(varargin{:});
    end
    
    % LOAD EXPERIMENT META DATA -------------------------------------------
    expPath = options.path;    
    expInfo = getExperimentInfo(expPath);
    
    % Setup default values for missing options
    if ~isfield(options, 'name'); options.name = '<unknown>'; end
    if ~isfield(options, 'showRawData'); options.showRawData = false; end
    if ~isfield(options, 'doCalibration'); options.doCalibration = false; end
    if ~isfield(options, 'showGps'); options.showGps = false; end
    if ~isfield(options, 'doEkf'); options.doEkf = false; end
    if ~isfield(options, 'doCorrection'); options.doCorrection = true; end
    if ~isfield(options, 'calibrationFile')
        options.calibrationFile = fullfile(fileparts(expPath), 'calibration.mat'); 
    end
    if ~isfield(options, 'disableBiasCompensation'); options.disableBiasCompensation = false; end
    if ~isfield(options, 'savePdfs'); options.savePdfs = false; end
    
    hFigures = [];        
    
    fprintf('============================================================\n');
    fprintf('Experiment: %s\n(%s)\n', expInfo.name, options.path);
    fprintf('============================================================\n');
    
    % LOAD EXPERIMENT SENSOR DATA -----------------------------------------
    dataPath = fullfile(expPath, 'sensor_data');
    accDataFile = fullfile(dataPath, 'accelerometer_data.dat');
    if exist(accDataFile, 'file')
        accelerometer_data = importSensorData(accDataFile);
        
        a.samples    = size(accelerometer_data,1);
        a.ID         = (accelerometer_data(:,1))';
        a.time       = (accelerometer_data(:,2))'.*10^(-9);
        a.value      = (accelerometer_data(:,3:5))';
        
        a.name       = 'Measured Body Acceleration';
        a.axis1      = '${}^{\rm{B}}\tilde{a}_{\rm{IB},x}$ / $\frac{\rm{m}}{\rm{s}^2}$';
        a.axis2      = '${}^{\rm{B}}\tilde{a}_{\rm{IB},y}$ / $\frac{\rm{m}}{\rm{s}^2}$';
        a.axis3      = '${}^{\rm{B}}\tilde{a}_{\rm{IB},z}$ / $\frac{\rm{m}}{\rm{s}^2}$';
        a.base       = '$t$ / $\rm{s}$';        
        
    else accelerometer_data = [];
    end
    gyroDataFile = fullfile(dataPath, 'gyroscope_data.dat');
    if exist(gyroDataFile, 'file')
        gyroscope_data = importSensorData(gyroDataFile);

        w.samples    = size(gyroscope_data,1);
        w.ID         = (gyroscope_data(:,1))';
        w.time       = (gyroscope_data(:,2))'.*10^(-9);
        w.value      = (gyroscope_data(:,3:5))';            
        
        w.name       = 'Measured Body Angular Rate';
        w.axis1      = '${}^{\rm{B}}\tilde{\omega}_{\rm{IB},x}$ / $\frac{\rm{rad}}{\rm{s}}$';
        w.axis2      = '${}^{\rm{B}}\tilde{\omega}_{\rm{IB},y}$ / $\frac{\rm{rad}}{\rm{s}}$';
        w.axis3      = '${}^{\rm{B}}\tilde{\omega}_{\rm{IB},z}$ / $\frac{\rm{rad}}{\rm{s}}$';
        w.base       = '$t$ / $\rm{s}$';
        
    else gyroscope_data = [];
    end
    gpsDataFile = fullfile(dataPath, 'gps_data.dat');
    if exist(gpsDataFile, 'file')
        gps_data = importSensorData(gpsDataFile);
        
        r.WGS.samples    = size(gps_data,1);
        r.WGS.ID         = (gps_data(:,1))';
        r.WGS.time       = (gps_data(:,2))'.*10^(-9);
        r.WGS.value      = (gps_data(:,3:5))';
        
        r.WGS.name       = 'Measured Body Position (Geodetic)';
        r.WGS.axis1      = '$\tilde{\phi}_{\rm{B}}$ / ${}^{\circ}$';
        r.WGS.axis2      = '$\tilde{\lambda}_{\rm{B}}$ / ${}^{\circ}$';
        r.WGS.axis3      = '$\tilde{h}_{\rm{B}}$ / $\rm{m}$';
        r.WGS.base       = '$t$ / $\rm{s}$';
        
        % Initialize Navigation Frame
        r.N.ECEFpos(:,1) = WGSpos2ECEFpos(r.WGS.value(:,1));
        r.N.ECEFrot(:,1) = WGSpos2NEDquat(r.WGS.value(:,1));

        r.N.samples    = r.WGS.samples;
        r.N.ID         = r.WGS.ID ;
        r.N.time       = r.WGS.time;

        for i = 1:r.WGS.samples
            r.ECEF.value(:,i) = WGSpos2ECEFpos(r.WGS.value(:,i));

            r.N.value(:,i)    = (Quat2DCM(r.N.ECEFrot))' * (r.ECEF.value(:,i) - r.N.ECEFpos);
            r.N.sigma(1,i)    =   gps_data(i,8);
            r.N.sigma(2,i)    =   gps_data(i,8);
            r.N.sigma(3,i)    = 2*gps_data(i,8);
            r.N.name          = 'Measured Body Position (in Navigation Frame)';
        end        
        
    else gps_data = [];
    end
    
    % READ SENSOR CALIBRATION DATA FROM FILE ------------------------------
    calibFields = {'a_sigma', 'a_bias', 'w_sigma', 'w_bias'};
    try
        calib = load(options.calibrationFile);
        for field = calibFields
            if isfield(calib, field{1})
                fprintf('Using bias.%s = [%16f %16f %16f]\n', field{1}, calib.(field{1})(1), calib.(field{1})(2), calib.(field{1})(3));
            else warning('project:bias:MissingField', 'Bias file %s does not contain bias "%s" - using zero default', options.calibrationFile, field{1});
            end
        end        
    catch e
        warning('project:bias:LoadError', 'Could not load bias values from %s: %s,\nUsing zero default bias', options.calibrationFile, e.message);
        calib = struct();
    end
            
    % PERFORM SENSOR CALIBRATION ------------------------------------------
    if options.doCalibration
        calibRecords = {};
        
        if ~isempty(accelerometer_data)
            [calib.a_sigma, a_mean, calib.a_bias] = aCalibration(a.value);
            dumpCalibrationData('Accelerometer', calib.a_sigma, a_mean, calib.a_bias);            
            a.mean = repmat(a_mean(:), 1, a.samples);
            a.sigma = repmat(calib.a_sigma(:), 1, a.samples);            
            calibRecords{end + 1} = sprintf('- Accelerometer bias  = [%10f %10f %10f]\n', calib.a_bias(1), calib.a_bias(2), calib.a_bias(3));
            calibRecords{end + 1} = sprintf('- Accelerometer sigma = [%10f %10f %10f]\n', calib.a_sigma(1), calib.a_sigma(2), calib.a_sigma(3));
        end
        if ~isempty(gyroscope_data)
            [calib.w_sigma, w_mean, calib.w_bias] = wCalibration(w.value);
            dumpCalibrationData('Gyroscope', calib.w_sigma, w_mean, calib.w_bias);            
            w.mean = repmat(w_mean(:), 1, w.samples);
            w.sigma = repmat(calib.w_sigma(:), 1, w.samples);
            calibRecords{end + 1} = sprintf('- Gyroscope bias      = [%10f %10f %10f]\n', calib.w_bias(1), calib.w_bias(2), calib.w_bias(3));
            calibRecords{end + 1} = sprintf('- Gyroscope sigma     = [%10f %10f %10f]\n', calib.w_sigma(1), calib.w_sigma(2), calib.w_sigma(3));
        end
        
        if ~isempty(calibRecords)
            choice = questdlg([sprintf('Save the calibration results\n') calibRecords{:} 'to ' options.calibrationFile '?'], 'Bias Calibration', 'Yes', 'No', 'Yes');
            if strcmp(choice, 'Yes')
                try
                    save(options.calibrationFile, '-struct', 'calib');
                catch e
                    warning('project:bias:SaveError', 'Could not save bias data to %s: %s.\n', options.calibrationFile, e.message);
                end
            end
        end
    end
    
    % provide zero defaults
    for field = calibFields(~isfield(calib, calibFields))
        calib.(field{1}) = [0; 0; 0];
    end
    
    % SHOW RAW SENSOR DATA ------------------------------------------------
    if options.showRawData
        if ~isempty(accelerometer_data)
            addInstanceFigures(plotErrors(applyDisplaySettings(a)));
        end        
        if ~isempty(gyroscope_data)            
            addInstanceFigures(plotErrors(applyDisplaySettings(w)));
        end
        if ~isempty(gps_data);
            r.WGS.mean = repmat(mean(gps_data(:, 3:5))', 1, r.WGS.samples);            
            r.WGS.sigma = repmat(std(gps_data(:, 3:5))', 1, r.WGS.samples);
            addInstanceFigures(plotErrors(applyDisplaySettings(r.WGS)));
        end
    end
    
    % SHOW TRAJECTORY FROM RAW GPS DATA -----------------------------------
    if options.showGps
        if ~isempty(gps_data)
            traj3dWnd = trajectory3dWindow('3D GPS trajectory');
            
            traj3dWnd.setData(r.N.time, r.N.value, repmat([0; 0; 0; 1], 1, r.N.samples));
            addInstanceFigures(traj3dWnd.handle);
            
        else warn('The Experiment does not contain GPS data.');
        end
    end    

    
    % LOCALIZATION FILTER -------------------------------------------------            
    doLoop = true;
    if options.doEkf
        if ~isempty(gps_data) && ~isempty(accelerometer_data) && ~isempty(gyroscope_data)            
            
            filter3dWnd = trajectory3dWindow('EKF Filter: 3D Position & Attitude');            
            addInstanceFigures(filter3dWnd.handle);
            
            t_max = max([a.time(end), w.time(end), r.N.time(end)]);
            progressDlg = waitbar(0, 'EKF computation...', 'CreateCancelBtn', @cancelEkf);
            
            % Initialization
            d_D = -a.value(:, 1) / norm(a.value(:, 1));
            d_N = [0; 0; 1];
            d_E = -cross(d_N, d_D);
            d_E = d_E / norm(d_E);
            d_N = cross(d_E, d_D);

            R = [d_N d_E d_D]';

            x = [r.N.value(:, 1); expInfo.initialVelocity(:)];
            if isfield(expInfo, 'initialPose')
                x = [x; expInfo.initialPose(:)];
            else x = [x; DCM2Quat(R)];
            end
            P = diag([r.N.sigma(:, i); 1; 1; 1; 0.5; 0.5; 0.5; 0.5].^2);
            
            maxRecords = a.samples + w.samples + r.N.samples;
            % preallocation
            t = zeros(1, maxRecords);
            pos.time = t;
            pos.value = zeros(3, maxRecords);
            pos.sigma = zeros(3, maxRecords);
            vel.time = t;
            vel.value = zeros(3, maxRecords);
            vel.sigma = zeros(3, maxRecords);
            rot.time = t;
            rot.value = zeros(5, maxRecords);
            rot.sigma = zeros(4, maxRecords);            
            t(1)  = r.N.time(1);

            i = 1;            
            ia = 1;
            iw = 1;
            ir = 1;            
            lastUpdate = 0;
            
            while doLoop
                % log current values
                p = sqrt(diag(P));

                pos.time(1, i)  = t(1,i);
                pos.value(:, i) = x(1:3);
                pos.sigma(:, i) = p(1:3);

                vel.time(1, i)  = t(1,i);
                vel.value(:, i) = x(4:6);
                vel.sigma(:, i) = p(4:6);

                rot.time(1, i)  = t(1, i);
                rot.value(1:4, i) = x(7:10);
                rot.value(5, i) = norm(x(7:10));
                rot.sigma(:, i) = p(7:10);
                
                if ia == a.samples || iw == w.samples || ir == r.N.samples
                    doLoop = false;
                end
                
                if ~doLoop || (i - lastUpdate) >= 100
                    filter3dWnd.addData(pos.time(1, (lastUpdate + 1):i), pos.value(:, (lastUpdate + 1):i), rot.value(1:4, (lastUpdate + 1):i));
                    lastUpdate = i;
                    
                    waitbar(t(i) / t_max, progressDlg);
                    drawnow();
                end
                
                % End of update - leave here, if all data has been
                % processed or the user pressed 'cancel'
                if ~doLoop; break; end                
                
                i = i + 1;                
                                
                % find current acceleration
                while 1
                    if a.time(ia) <= t(i - 1) && a.time(ia + 1) >= t(i - 1)
                        break;
                    else
                        ia = ia + 1;
                    end
                end

                % find current angular rate
                while 1
                    if w.time(iw) <= t(i - 1) && w.time(iw + 1) >= t(i - 1)
                        break;
                    else
                        iw = iw + 1;
                    end
                end

                % find current position measurement
                while 1
                    if r.N.time(ir) <= t(i - 1) && r.N.time(ir + 1) >= t(i - 1)
                        break;
                    else
                        ir = ir + 1;
                    end
                end

                % Input Vector
                u = [a.value(:, ia); w.value(:, iw)];
                if expInfo.biasCorrection && ~options.disableBiasCompensation
                    u = u - [calib.a_bias; calib.w_bias];
                end
                % Input Covariance (sigma results from calibrations are
                % enlarged by a manually-adjusted factor)
                N = System_N(calib.a_sigma, 20 * calib.w_sigma);                
                
                % find next time step
                t(i) = t(i - 1);
                while t(i) == t(i - 1)
                    [t(i), sensorIndex] = min([a.time(ia + 1); w.time(iw + 1); r.N.time(ir + 1)]);

                    switch sensorIndex
                        case 1; ia = ia + 1; % accelerometer
                        case 2; iw = iw + 1; % gyro
                        case 3; ir = ir + 1; % gps
                    end
                end

                % prediction step
                [x, P] = doPrediction(x, P, u, N, t(i) - t(i - 1));

                % correction step
                if options.doCorrection && sensorIndex == 3
                    [x, P] = doCorrection(x, P, r.ECEF.value(:, ir), r.WGS.value(:, ir), r.N.sigma(:, ir), r.N.ECEFpos, r.N.ECEFrot);
                end                
            end            
            
            delete(progressDlg);
            
            % prepare result data figures...
            pos.time((i + 1):end) = [];
            pos.value(:, (i + 1):end) = [];
            pos.sigma(:, (i + 1):end) = [];
            pos.name       = 'Estimated Body Position (in Navigation Frame)';
            pos.axis1      = '${}^{\rm{N}}\hat{r}_{\rm{NB},x}$ / $\rm{m}$';
            pos.axis2      = '${}^{\rm{N}}\hat{r}_{\rm{NB},y}$ / $\rm{m}$';
            pos.axis3      = '${}^{\rm{N}}\hat{r}_{\rm{NB},z}$ / $\rm{m}$';
            pos.base       = '$t$ / $\rm{s}$';

            vel.time((i + 1):end) = [];
            vel.value(:, (i + 1):end) = [];
            vel.sigma(:, (i + 1):end) = [];
            vel.name       = 'Estimated Body Velocity (in Navigation Frame)';
            vel.axis1      = '${}^{\rm{N}}\hat{v}_{\rm{NB},x}$ / $\rm{m}$';
            vel.axis2      = '${}^{\rm{N}}\hat{v}_{\rm{NB},y}$ / $\rm{m}$';
            vel.axis3      = '${}^{\rm{N}}\hat{v}_{\rm{NB},z}$ / $\rm{m}$';
            vel.base       = '$t$ / $\rm{s}$';            
            
            rot.time((i + 1):end) = [];
            rot.value(:, (i + 1):end) = [];
            rot.sigma(:, (i + 1):end) = [];
            rot.name       = 'Estimated Body Attitude (wrt Navigation Frame)';
            rot.axis1      = '${}^{\rm{N}}_{\rm{B}}\hat{q}_{1}$ / $1$';
            rot.axis2      = '${}^{\rm{N}}_{\rm{B}}\hat{q}_{2}$ / $1$';
            rot.axis3      = '${}^{\rm{N}}_{\rm{B}}\hat{q}_{3}$ / $1$';
            rot.axis4      = '${}^{\rm{N}}_{\rm{B}}\hat{q}_{4}$ / $1$';
            rot.axis5      = '$|{}^{\rm{N}}_{\rm{B}}\hat{q}|$ / $1$';
            rot.base       = '$t$ / $\rm{s}$';            
            
            % ...and show them
            addInstanceFigures([plotErrors(applyDisplaySettings(pos), r.N), ...
                                plotErrors(applyDisplaySettings(vel)), ...
                                plotErrors(applyDisplaySettings(rot))]);                    
        
        else warning('project:ekf:MissingSensor', 'Cannot run EKF filter: Either accelerometer data or gyroscope data or GPS data is missing.');
        end        
    end

    % --- internal functions ---
    
    function cancelEkf(varargin)
        doLoop = false;
    end
    function s = applyDisplaySettings(s)
        if options.savePdfs
            s.pdffile = [expInfo.id '_' s.name];
        end
    end
    function addInstanceFigures(h)
        hFigures = [hFigures; h(:)];
        set(h, 'CloseRequestFcn', @(varargin)closeFigures);
        function closeFigures(varargin)
            doLoop = false;
            delete(hFigures);            
            hFigures = [];
        end        
    end
    function dumpCalibrationData(sensorName, x_sigma, x_mean, x_bias)
        fprintf([sensorName ' calibration data:\n']);
        fprintf(' Sigma = [%16f, %16f, %16f]\n', x_sigma(1), x_sigma(2), x_sigma(3));
        fprintf(' Mean  = [%16f, %16f, %16f]\n', x_mean(1), x_mean(2), x_mean(3));
        fprintf(' Bias  = [%16f, %16f, %16f]\n', x_bias(1), x_bias(2), x_bias(3));
    end
end

% Generic implementation of the EKF prediction step
function [x, P] = doPrediction(x, P, u, N, deltaT)
    g = 9.81;

    % Linearization
    A = System_A(x, u, g);
    B = System_B(x, u, g);

    % Discretization
    [F, H] = discretize(A, B, deltaT);

    % State & Covariance Propagation
    [~, X] = ode45(@(t, x) System_dx(t, x, u, g), [0, deltaT], x, odeset('RelTol', 10e-13, 'AbsTol', eps));

    x = (X(end, :)).';
    P = F * P * F.' + H * N * H.';       
end

% Generic implementation of the EKF correction (update) step
function [x, P] = doCorrection(x, P, y_meas, WGS_pos, NED_sigma, r_ECEF_N, q_ECEF_N)
    % Output Covariance
    W      = System_W(WGS_pos, NED_sigma);

    % Output Matrix
    C      = System_C(x, r_ECEF_N, q_ECEF_N);

    % Output Prediction
    y_pred = System_y(x, r_ECEF_N, q_ECEF_N);

    % y_meas = Output Measurement

    % Observability 
    %ObsDefect  = length(x) - rank(obsv(A,C));

    % Kalman Gain
    K  = P * C.' / (C * P * C.' + W);

    % State & Covariance Correction
    x  = x + K * (y_meas - y_pred); 
    P  = (eye(size(K,1)) - K * C) * P;

end
