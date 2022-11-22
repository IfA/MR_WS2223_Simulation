function s = getExperimentInfo(path, probeMode)
    if nargin < 2; probeMode = false; end
    infoFile = fullfile(path, 'experiment_info.xml');
    
    try
        s = struct('path', path, 'name', '', 'comment', '', 'id', '', 'biasCorrection', true, 'initialVelocity', [0; 0; 0]);
        doc = xmlread(infoFile);
        root = doc.getDocumentElement();
        
        if strcmp(root.getNodeName, 'experiment')
            nodes = root.getChildNodes;
            for i = 0:(nodes.getLength - 1)
                node = nodes.item(i);
                nodeName = char(node.getNodeName);
                switch nodeName
                    case 'bias_correction'
                        corr = char(node.getTextContent);
                        if strcmp(corr, 'false'); s.biasCorrection = false; end
                    case 'initial_velocity'
                        subs = node.getChildNodes;
                        for j = 0:(subs.getLength - 1)
                            sub = subs.item(j);                            
                            switch char(sub.getNodeName)
                                case 'x'; s.initialVelocity(1) = str2double(sub.getTextContent);
                                case 'y'; s.initialVelocity(2) = str2double(sub.getTextContent);
                                case 'z'; s.initialVelocity(3) = str2double(sub.getTextContent);
                            end                            
                        end
                    case 'initial_pose'
                        pose = [1; 0; 0; 0];
                        subs = node.getChildNodes;
                        for j = 0:(subs.getLength - 1)
                            sub = subs.item(j);
                            switch char(sub.getNodeName)
                                case 'q1'; pose(1) = str2double(sub.getTextContent);
                                case 'q2'; pose(2) = str2double(sub.getTextContent);
                                case 'q3'; pose(3) = str2double(sub.getTextContent);
                                case 'q4'; pose(4) = str2double(sub.getTextContent);
                            end
                        end
                        s.initialPose = pose;
                    otherwise
                        nodeChildren = node.getChildNodes();
                        if nodeChildren.getLength == 1;
                            textNode = nodeChildren.item(0);
                            if ~textNode.hasChildNodes()
                                s.(nodeName) = char(textNode.getTextContent());
                            end
                        end
                end
            end            

            sensorPath = fullfile(path, 'sensor_data');
            s.acc = (exist(fullfile(sensorPath, 'accelerometer_data.dat'), 'file') && exist(fullfile(sensorPath, 'accelerometer_info.txt'), 'file'));
            s.gyro = (exist(fullfile(sensorPath, 'gyroscope_data.dat'), 'file') && exist(fullfile(sensorPath, 'gyroscope_info.txt'), 'file'));
            s.gps = logical(exist(fullfile(sensorPath, 'gps_data.dat'), 'file'));
            s.camera = false;
            return;
        else
            if ~probeMode
                warning('project:exp:InfoFormat', 'Invalid project Description XML file');
            end
        end         
    catch e
        if ~probeMode
            warning('project:exp:Info', 'Error reading info for experiment %s: %s\n', path, e.message)        
        end
    end    
    s = [];        
end
