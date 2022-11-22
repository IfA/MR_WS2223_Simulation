function w = trajectory3dWindow(title)
    
    w.handle = figure('toolbar', 'figure', 'NumberTitle', 'off', 'Name', title);%, 'ResizeFcn', @(varargin)updateLayout());%, 'Renderer', 'OpenGl');  
    w.ax = axes('Units', 'pixels', 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', 'XDir', 'reverse', 'ZDir', 'reverse');
	camlight('headlight');
	axis equal;
    
    xlabel('North / m');
    ylabel('East / m');
    zlabel('Down / m');
    w.setData = @setData;
    w.addData = @addData;
    
    w.hTrack = line('Parent', w.ax, 'Color', [0 0 0], 'XData', [], 'YData', [], 'ZData', []);
    triadeObj = triade();
    triadeObj.hide();
    
    hPrevBtn = uicontrol('Parent', w.handle, 'style', 'pushButton', 'String', 'prev', 'Units', 'pixels', 'Callback', @(varargin)prev(), 'Enable', 'off');
    hSlider = uicontrol('Parent', w.handle, 'style', 'slider', 'Units', 'pixels', 'Min', 0, 'Max', 1, 'Value', 0, 'Callback', @(varargin)updateSliderPos(), 'Enable', 'off');
    hNextBtn = uicontrol('Parent', w.handle, 'style', 'pushButton', 'String', 'next', 'Units', 'pixels', 'Callback', @(varargin)next(), 'Enable', 'off');
    hTimeLabel = uicontrol('Parent', w.handle, 'style', 'text', 'Units', 'pixels', 'HorizontalAlignment', 'left', 'String', ' ---', 'Enable', 'off');
    hAttitudeCheck = uicontrol('Parent', w.handle, 'style', 'checkbox', 'Units', 'pixels', 'String', 'Show Attitude', 'Callback', @(varargin)updateAttitudeVisibility(), 'Enable', 'off', 'Value', 1);
    
    recordIndex = [];
    
    z = zoom(w.handle);
    set(z, 'ActionPostCallback', @(varargin)updateSliderPos());    
    
    updateLayout();
    view([90 90]);
    
    rotate3d();
    
    t = zeros(1, 0);
    r = zeros(3, 0);
    q = zeros(4, 0);
    
    function prev()
        if recordIndex > 1
            setRecordIndex(recordIndex - 1);
        end
    end
    function next()
        if recordIndex < numel(t)
            setRecordIndex(recordIndex + 1);            
        end
    end

    function updateLayout()
        pos = get(w.handle, 'Position');
        height = pos(4); width = pos(3);
        sliderHeight = 16;
        set(w.ax, 'OuterPosition', [0, 16, width, height - sliderHeight]);
        timeLabelWidth = 100;
        btnWidth = 44;
        set(hPrevBtn, 'Position', [0, 0, btnWidth, sliderHeight]);
        set(hSlider, 'Position', [btnWidth, 0, width - timeLabelWidth - 2 * btnWidth, sliderHeight]);
        set(hNextBtn, 'Position', [width - timeLabelWidth - btnWidth, 0, btnWidth, sliderHeight]);
        set(hTimeLabel, 'Position', [width - timeLabelWidth, 0, timeLabelWidth, sliderHeight]);        
        set(hAttitudeCheck, 'Position', [0, sliderHeight, 120, 16]);
    end
    function updateSliderPos()
        if isempty(t)
            set([hPrevBtn, hSlider, hNextBtn, hTimeLabel, hAttitudeCheck], 'Enable', 'off');
        else set([hPrevBtn, hSlider, hNextBtn, hTimeLabel], 'Enable', 'on');
        end
        
        tDesired = get(hSlider, 'Value');
        setRecordIndex(find(t <= tDesired, 1, 'last'));        
    end
    function setRecordIndex(idx)
        if idx < 1 || idx > numel(t)
            idx = [];
        end
        recordIndex = idx;
        if ~isempty(idx)
            set(hTimeLabel, 'String', sprintf(' t = %0.3f s', t(idx))); 
            set(hSlider, 'Value', t(recordIndex));
            xLim = get(w.ax, 'XLim'); yLim = get(w.ax, 'YLim'); zLim = get(w.ax, 'ZLim');
            triadeScale = 0.2 * min([abs(xLim(2) - xLim(1)), abs(yLim(2) - yLim(1)), abs(zLim(2) - zLim(1))]);
            
            triadeObj.place(T_shift(r(:, idx)) * T_rot('q', [q(4, idx) q(1, idx) q(2, idx) q(3, idx)]) * T_scale(triadeScale));            
            set(hAttitudeCheck, 'Enable', 'on');
            updateAttitudeVisibility();
        else
            triadeObj.hide();
            set(hAttitudeCheck, 'Enable', 'off');
            set(hTimeLabel, 'String', ' ---');
        end        
    end

    function updateAttitudeVisibility()
        if get(hAttitudeCheck, 'Value')
            triadeObj.show();
        else triadeObj.hide();
        end
    end

    function updateCurve()
        set(w.hTrack, 'XData', r(1, :), 'YData', r(2, :), 'ZData', r(3, :));
    end
    
    function setData(t_new, r_new, q_new)
        t = t_new;
        r = r_new;
        q = q_new;
        updateCurve();
        if isempty(t)
            set(hSlider, 'Min', 0, 'Max', eps, 'Value', 0);
        else set(hSlider, 'Min', t(1), 'Max', t(end), 'Value', t(end));        
        end
        updateSliderPos();
    end
    function addData(t_add, r_add, q_add)
        t = [t, t_add];
        r = [r, r_add];
        q = [q, q_add];
        updateCurve();
        set(hSlider, 'Min', t(1), 'Max', t(end), 'Value', t(end));        
        
        updateSliderPos();        
    end
end