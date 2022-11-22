% TUD/IfA, Course "Mobile Robotic" - Practical Project
%
% Experiment Selection & Launcher GUI
%
% Parameters:
% - initialPath (optional): working Directory preset. If omitted, the
%                           current directory is used)
% Return values:
% - f_out: handle to launcher figure
%
function f_out = launcher(initialPath) 
	
    defaultBackground = get(0, 'defaultUicontrolBackgroundColor');
    f = figure('NumberTitle', 'off', 'Color', defaultBackground, 'Name', 'Course: Mobile Robot Systems - Practical Project', 'toolbar', 'none', 'menubar', 'none');
    set(f,'ResizeFcn', @updateLayout);
    if nargout > 0
        f_out = f;
    end
           
    % Set window icon
    warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');	           
    jframe = get(f, 'javaframe');
    jframe.setFigureIcon(javax.swing.ImageIcon('appicon.png'));	
           
    
    hSelFrame = uibuttongroup('Title', 'Experiment Selection', 'Units', 'pixels', 'SelectionChangeFcn', @selOptionChanged);
    hDetailsFrame = uibuttongroup('Title', 'Details', 'Units', 'pixels');
    hTaskFrame = uibuttongroup('Title', 'Tasks', 'Units', 'pixels');
    hOptionsFrame = uibuttongroup('Title', 'Options', 'Units', 'pixels');
    
    hDirCaption = uicontrol('Parent', hSelFrame, 'style', 'text', 'String', 'Working Directory', 'units', 'pixels', 'HorizontalAlignment', 'left');
    hDirText = uicontrol('Parent', hSelFrame, 'style', 'edit', 'units', 'pixels', 'HorizontalAlignment', 'left', 'Callback', @(h, varargin)setExpDir(get(h, 'String')));
    hDirSelButton = uicontrol('Parent', hSelFrame, 'style', 'pushbutton', 'String', '...', 'units', 'pixels', 'Callback', @(varargin)selDirDialog());
    hExperimentList = uicontrol('Parent', hSelFrame, 'style', 'listbox', 'String', {}, 'Units', 'pixels', 'Callback', @(varargin)updateExperimentSelection());        
    hZipFileSelButton = uicontrol('Parent', hSelFrame, 'style', 'pushbutton', 'String', '*.zip', 'units', 'pixels', 'Callback', @(varargin)selZipFileDialog());
    hZipHelpLabel = uicontrol('Parent', hSelFrame, 'style', 'text', 'String', 'Extract a *.zip file into the working directory', 'units', 'pixels');
    
    hNameCaptionText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', 'Name', 'FontWeight', 'bold', 'Units', 'pixels', 'HorizontalAlignment', 'left');
    hNameText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', '---', 'units', 'pixels', 'HorizontalAlignment', 'left');
    hSensorsCaptionText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', 'Sensors', 'FontWeight', 'bold', 'units', 'Pixels', 'HorizontalAlignment', 'left');
    hSensorsText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', '', 'units', 'Pixels', 'HorizontalAlignment', 'left');
    hExpPathCaptionText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', 'Experiment Path', 'FontWeight', 'bold', 'units', 'Pixels', 'HorizontalAlignment', 'left');
    hExpPathText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', '$(home)/somewhere/exp', 'units', 'Pixels', 'HorizontalAlignment', 'left');
    hDescriptionCaptionText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', 'Description', 'FontWeight', 'bold', 'Units', 'pixels', 'HorizontalAlignment', 'left');
    hDescriptionText = uicontrol('Parent', hDetailsFrame, 'Style', 'text', 'String', '---', 'units', 'pixels', 'HorizontalAlignment', 'left');    
    
    hTaskButtons(1) = uicontrol('Parent', hTaskFrame, 'style', 'pushbutton', 'String', 'Sensor Calibration', 'Units', 'pixels', 'Callback', @(varargin)runCalibration()); 
    hTaskButtons(2) = uicontrol('Parent', hTaskFrame, 'style', 'pushbutton', 'String', 'GPS only', 'Units', 'pixels', 'Callback', @(varargin)runGpsOnly()); 
    hTaskButtons(3) = uicontrol('Parent', hTaskFrame, 'style', 'pushbutton', 'String', 'IMU only', 'Units', 'pixels', 'Callback', @(varargin)runImuOnly()); 
    hTaskButtons(4) = uicontrol('Parent', hTaskFrame, 'style', 'pushbutton', 'String', 'EKF Navigation', 'Units', 'pixels', 'Callback', @(varargin)runEkf()); 
    
    hRawDataCheck = uicontrol('Parent', hOptionsFrame, 'style', 'checkbox', 'String', 'Show raw Acc + Gyro data', 'Value', 1, 'Units', 'pixels');
    hUseBiasCheck = uicontrol('Parent', hOptionsFrame, 'style', 'checkbox', 'String', 'Use Bias Compensation', 'Value', 1, 'Units', 'pixels');
    hSavePdfsCheck = uicontrol('Parent', hOptionsFrame, 'style', 'checkbox', 'String', 'Save figures as PDFs', 'Value', 0, 'Units', 'pixels');
    
    if nargin >= 1
        expDirPath = initialPath;
    else expDirPath = pwd();
    end
    experimentInfos = {};
    experimentInfo = [];
    
    updateLayout();    
    
    setExpDir(expDirPath);    
    if ~isempty(expDirPath)
        updateExperimentSelection();
    else
        setExperiment([]);
        selDirDialog();
    end
    
    function updateLayout(varargin)
        try 
            [w, h] = getSize(f);

            pad = 4;
            titlePad = 16;

            y = pad;
            checkHeight = 16;
            optionsHeight = pad + titlePad + checkHeight;
            set(hOptionsFrame, 'Position', [pad, y, w - 2 * pad, optionsHeight]);
            y = y + optionsHeight + pad;

            buttonHeight = 32;        
            taskHeight = buttonHeight + titlePad + pad;                    
            set(hTaskFrame, 'Position', [pad, y, w - 2 * pad, taskHeight]);
            y = y + taskHeight + pad;

            hSpace = max(1, w - 3 * pad);
            listW = round(1/2 * hSpace);
            set(hSelFrame, 'Position', [pad, y, listW, h - y - pad]);
            set(hDetailsFrame, 'Position', [2 * pad + listW, y, max(1, w - 3 * pad - listW), h - y - pad]);

            % Experiment selection frame
            [w, h] = getSize(hSelFrame);
            y = pad;
            set(hZipFileSelButton, 'Position', [pad, y, 40, 24]);
            set(hZipHelpLabel, 'Position', [2 * pad + 40, y + 4, w - 3 * pad - 40, 16]);
            y = y + 24 + pad;
            boxHeight = h - y - 2 * pad - titlePad - 16 - 24;
            set(hExperimentList, 'Position', [pad, y, w - 2 * pad, boxHeight]);
            y = y + boxHeight + pad;
            set(hDirText, 'Position', [pad, y, w - 3 * pad - 24, 24]);
            set(hDirSelButton, 'Position', [w - pad - 24, y, 24, 24]);
            y = y + pad + 24;
            set(hDirCaption, 'Position', [pad, y, w - 2 * pad, 16]);

            % Details Frame
            [w, h] = getSize(hDetailsFrame);
            hCapHeight = 16;
            hLineHeight = 16;
            y = h - titlePad - hCapHeight;
            set(hNameCaptionText, 'Position', [pad, y, w - 2 * pad, hCapHeight]);
            y = y - hLineHeight - pad;
            set(hNameText, 'Position', [pad, y, w - 2 * pad, hLineHeight]);
            y = y - hCapHeight - pad;
            set(hSensorsCaptionText, 'Position', [pad, y, w - 2 * pad, hCapHeight]);
            y = y - hLineHeight - pad;
            set(hSensorsText, 'Position', [pad, y, w - 2 * pad, hLineHeight]);
            y = y - hCapHeight - pad;
            set(hExpPathCaptionText, 'Position', [pad, y, w - 2 * pad, hCapHeight]);
            y = y - 3 * hLineHeight - pad;
            set(hExpPathText, 'Position', [pad, y, w - 2 * pad, 3 * hLineHeight]);
            y = y - hCapHeight - pad;
            set(hDescriptionCaptionText, 'Position', [pad, y, w - 2 * pad, hCapHeight]);
            set(hDescriptionText, 'Position', [pad, pad, w - 2 * pad, y - pad]);        


            % Task Frame
            [w, ~] = getSize(hTaskFrame);
            buttonWidths = repmat(120, size(hTaskButtons));
            buttonWidths(1) = 160;
            optiWidth = sum(buttonWidths) + pad * (numel(buttonWidths) + 1);
            if optiWidth > w
                offset = 0;
                scaleFactor = (w - pad * (1 + numel(hTaskButtons))) / sum(buttonWidths);
            else
                offset = (w - optiWidth) / 2;
                scaleFactor = 1;
            end
            buttonStarts = [0, cumsum(buttonWidths)];
            for i = 1:numel(hTaskButtons)
                set(hTaskButtons(i), 'Position', [offset + i * pad + buttonStarts(i) * scaleFactor, pad, buttonWidths(i) * scaleFactor, buttonHeight]);
            end
            
            % Options Frame
            x = pad;
            optionWidth = 180;
            set(hUseBiasCheck, 'Position', [x, pad, optionWidth, checkHeight]); x = x + optionWidth + pad;
            optionWidth = 190;
            set(hRawDataCheck, 'Position', [x, pad, optionWidth, checkHeight]); x = x + optionWidth + pad;
            optionWidth = 150;
            set(hSavePdfsCheck, 'Position', [x, pad, optionWidth, checkHeight]);
        catch e
            warning('project:ui:Layout', 'Layout error: %s', e.message);           
        end
    end  

    function selDirDialog() 
        res = uigetdir(get(hDirText, 'String'), 'Select Directory with experiments');
        
        if res == 0; return; end        
        setExpDir(res);
    end
    function loadExpDirContent(path, selection)
        if ~isempty(path)            
            entries = dir(path);
            nValid = 0;
            validDirs = cell(size(entries));
            experimentInfos = cell(size(entries));
            for i = 3:numel(entries)
                if entries(i).isdir
                    expInfo = getExperimentInfo(fullfile(path, entries(i).name), true);
                    if ~isempty(expInfo)
                        nValid = nValid + 1;
                        validDirs{nValid} = sprintf('%s (%s)', expInfo.name, expInfo.id);                        
                        experimentInfos{nValid} = expInfo;
                    end
                end
            end
            validDirs((nValid + 1):end) = [];
            experimentInfos((nValid + 1):end) = [];
            
            selIdx = 1;                
            if nargin >= 2
                for i = 1:length(experimentInfos)
                    if strcmp(experimentInfos{i}.path, selection)
                        selIdx = i;
                        break;
                    end
                end
            end
            set(hExperimentList, 'Value', selIdx, 'String', validDirs);            
            updateExperimentSelection();
        end        
    end
    function setExpDir(path)
        if isempty(path); return; end
        if ~exist(path, 'dir')
            msgbox('The selected directory does not exist', 'Experiment Directory Selection', 'error');
            set(hDirText, 'String', expDirPath);
            return;
        end
        expDirPath = path;
        set(hDirText, 'String', expDirPath);        

        % scanning for zip files
        zipfiles = dir(fullfile(path, '*.zip'));        
        entries = cell(size(zipfiles));
        nEntries = 0;
        for i = 1:numel(zipfiles)
            if ~zipfiles(i).isdir
                [~, name, ~] = fileparts(zipfiles(i).name);
                if ~exist(fullfile(path, name), 'dir')                    
                    nEntries = nEntries + 1;
                    entries{nEntries} = zipfiles.name;
                end
            end
        end
        if nEntries > 0
            choice = questdlg('The directory contains *.zip files. Do you want to extract them?', 'Unzip Experiments', 'Yes', 'No', 'No');
            if strcmp(choice, 'Yes')
                for i = 1:nEntries
                    [~, name, ~] = fileparts(entries{i});
                    extractExperiment(fullfile(path, entries{i}), fullfile(path, name));
                end        
            end
        end
        
        loadExpDirContent(expDirPath);
    end
    function updateExperimentSelection()
        index = get(hExperimentList, 'value');
        if ~isempty(index)
            if (index >= 1) && (index <= numel(experimentInfos))
                setExperiment(experimentInfos{index});            
                return;
            end
        end
        setExperiment([]);
    end

    function setExperiment(exp)
        experimentInfo = exp;
        if ~isempty(exp)
            set(hNameText, 'String', exp.name);
            set(hExpPathText, 'String', exp.path);
            set(hDescriptionText, 'String', exp.comment);    
            
            availableSensors = {};            
            if exp.acc; availableSensors{end + 1} = 'ACC'; end
            if exp.gyro; availableSensors{end + 1} = 'GYRO'; end
            if exp.gps; availableSensors{end + 1} = 'GPS'; end
            set(hSensorsText, 'String', private_strjoin(availableSensors, ' + '));
            
            set(hTaskButtons(1), 'Enable', boolToOnOff(exp.gps || exp.acc || exp.gyro));
            set(hTaskButtons(2), 'Enable', boolToOnOff(exp.gps));
            set(hTaskButtons(3), 'Enable', boolToOnOff(exp.acc && exp.gyro && exp.gps));
            set(hTaskButtons(4), 'Enable', boolToOnOff(exp.acc && exp.gyro && exp.gps));
        else
            set(hNameText, 'String', '---');
            set(hExpPathText, 'String', '');
            set(hDescriptionText, 'String', '---');
            set(hTaskButtons, 'Enable', 'off');
        end        
    end

    function b = extractExperiment(zipFile, destPath)
        b = false;
        if ~exist(destPath, 'dir')
            dialog = showProgressMessage(['Unzipping ' zipFile '...']);                
            try 
                unzip(zipFile, destPath);
                fprintf('Archive "%s" extracted\n', zipFile);
                b = true;
            catch e
                msgbox(['Error while unzipping "' zipFile '": ' e.message], 'Unzip operation', 'error');
            end
            dialog.close();
        end
    end

    function selZipFileDialog()
        [res, basePath] = uigetfile({'*.zip', 'Zip archives (*.zip)'}, 'Select Compressed Experiment', expDirPath);
        if res == 0; return; end
        [~, name, ~] = fileparts(res);
        dirPath = fullfile(basePath, name);
        if exist(dirPath, 'dir')
            choice = questdlg('A directory with the same name as the zip file already exists. Overwrite it?', 'Unzip Experiment', 'Yes', 'No', 'No');
            if strcmp(choice, 'No'); return; end
            if rmdir(dirPath, 's') == 0
                msgbox('Could not remove original directory', 'Unzip Experiment', 'error');
                return;
            end
        end
        if extractExperiment(fullfile(basePath, name), dirPath)
            loadExpDirContent(expDirPath, dirPath);
        end                    
    end

    function spec = buildProjectSpec(varargin)
        spec.path = experimentInfo.path;
        spec.showRawData = logical(get(hRawDataCheck, 'Value'));
        spec.disableBiasCompensation = ~logical(get(hUseBiasCheck, 'Value'));
        spec.savePdfs = logical(get(hSavePdfsCheck, 'Value'));
        additionalOpts = struct(varargin{:});        
        fNames = fieldnames(additionalOpts);
        for iField = 1:numel(fNames)
            spec.(fNames{iField}) = additionalOpts.(fNames{iField});
        end
    end
    function runCalibration() 
        project(buildProjectSpec('doCalibration', true));
    end
    function runGpsOnly()
        project(buildProjectSpec('showGps', true));
    end
    function runImuOnly()
        project(buildProjectSpec('doEkf', true, 'doCorrection', false));
    end
    function runEkf()
        project(buildProjectSpec('doEkf', true, 'doCorrection', true));
    end
end

function [w, h] = getSize(handle)
    pos = get(handle, 'Position');
    w = pos(3);
    h = pos(4);
end

function dialog = showProgressMessage(string, varargin)
    handle = msgbox(sprintf(string, varargin{:}), 'Operation in progress', 'Help');
    dialogChildren = get(handle, 'Children');
    delete(dialogChildren(end));
    set(handle, 'CloseRequestFcn', '');            
    dialog.close = @closeDialog;    
    function closeDialog()
        if ishghandle(handle); delete(handle); end
    end
end
function o = boolToOnOff(b)
    if b; o = 'on'; else o = 'off'; end
end
function s = private_strjoin(cellArr, delimiter) % The function strjoin is not available on older Matlab installations
    if isempty(cellArr)
        s = '';
    else
        arrWithDelims = [cellArr(1); reshape([repmat({delimiter}, numel(cellArr) - 1, 1), reshape(cellArr(2:end), [], 1)]', [], 1)];
        s = [arrWithDelims{:}];
    end
end
