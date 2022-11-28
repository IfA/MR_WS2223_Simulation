% This block implements common visualization code for landmark-based 
% Simultaneous Localization and Mapping (SLAM) algorithms. Use it as a
% "base class" for blocks that implement an acutal SLAM filter algorithm.
% Format of expected block inputs:
% - [platform] - irrelevant
% - [sensor] - irrelevant
% - landmarks - Nx2 array of landmark positions (usually from a const_point block)
% Expected output format: struct with fields
% - .featurePositions: Nx2 array of estimated landmark positions, one per row
% - .landmarkIds: feature-to-landmark associations, Nx1 column vector of indices into the landmark input
% - .featureCovariances: Nx3 array where each row [sigma_xx, sigma_xy, sigma_yy] describes the uncertainty of a landmark position estimation
% 
% An optional parameter [default_]featureSigmaScale may be added (by the
% subclassing block or in the experiment definition) to specify the size of
% the uncertainty ellipses of the landmarks. If it is not given, the value
% of sigmaScale from filter_localization2d will be used.

function filter = filter_slam2d(processFunction)
    filter = filter_localization2d(processFunction);
    filter.depends(end) = []; % remove feature map from inputs
	
    filter.graphicElements(end + 1).draw = @drawFeatures;
    filter.graphicElements(end).name = 'feature positions';    
    filter.graphicElements(end + 1).draw = @drawCovariances;
    filter.graphicElements(end).name = 'feature covariance ellipses';    
    filter.graphicElements(end + 1).draw = @drawLmAssociations;
    filter.graphicElements(end).name = 'landmark associations';    
    
    function handles = drawFeatures(block, ax, handles, out, debugOut, state, platform, varargin)        
        if isempty(handles) 
            handles = line('Parent', ax, 'XData', [], 'YData', [], 'Marker', '.', 'LineStyle', 'none', 'Color', block.color);
        end
        set(handles, 'XData', out.featurePositions(:, 1), 'YData', out.featurePositions(:, 2));
    end

    function handles = drawCovariances(block, ax, handles, out, debugOut, state, platform, varargin)        
        M = size(out.featurePositions, 1);
        if length(handles) < M
            startIdx = length(handles) + 1;
            handles = [handles, zeros(1, M - length(handles))];
            for i = startIdx:M
                handles(i) = patch('Parent', ax, 'XData', [], 'YData', [], 'FaceColor', 'none', 'EdgeColor', block.color);
            end
        elseif length(handles) > M
            delete(handles((M + 1):end));
            handles((M + 1):end) = [];
        end
        
		t = linspace(0, 2 * pi, 10);
		CS = [cos(t'), sin(t')];
		if isfield(block, 'featureSigmaScale'); sigmaScale = block.featureSigmaScale; else sigmaScale = block.sigmaScale; end
		
		for i = 1:M			
			cov = out.featureCovariances(i, :);
            [eigvec, eigval] = eig([cov(1), cov(2); cov(2), cov(3)]);
			XY = CS * diag(sigmaScale * sqrt(diag(eigval))) * eigvec';
			set(handles(i), 'XData', out.featurePositions(i, 1) + XY(:, 1), 'YData', out.featurePositions(i, 2) + XY(:, 2));
		end		        
    end

    function handles = drawLmAssociations(block, ax, handles, out, debugOut, state, platform, sensor, varargin)                
		% in case no reference landmarks are provided, we skip drawing the
		% feature<->landmark associations
		if length(varargin) >= 1
			landmarks = varargin{1};
		else return;
		end
		
        if isempty(handles)
            handles = line('Parent', ax, 'XData', [], 'YData', [], 'Color', block.color);
        end
        
        M = length(out.landmarkIds);
        coords = NaN(3 * M, 2);
        for i = 1:M
            lmId = out.landmarkIds(i);
            if lmId > 0 && lmId <= size(landmarks.data, 1)
                coords(3 * i, :) = landmarks.data(lmId, :);
                coords(3 * i + 1, :) = out.featurePositions(i, :);
            end
        end
        set(handles, 'XData', coords(:, 1), 'YData', coords(:, 2));
    end
end
