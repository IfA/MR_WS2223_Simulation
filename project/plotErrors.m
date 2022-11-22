function h = plotErrors( input1, input2, input3 )

    h = figure;
    if isfield(input1, 'name')
        set(h, 'numbertitle', 'off', 'name', input1.name);
    end
    
    n = size(input1.value,1);
    axs = zeros(1, n);
    for i = 1:n
        axs(i) = subplot(n,1,i); hold on; grid on;
        if isfield(input1,'name') && i == 1
            lab = title(input1.name); set(lab, 'interpreter', 'latex', 'FontSize', 14);
        end
        if nargin >= 1 
            plot(input1.time,input1.value(i,:),'b');
            xlim([input1.time(1,1),input1.time(1,end)+eps])
        end
        if nargin >= 2
            plot(input2.time,input2.value(i,:),'g');
            xlim([input2.time(1,1),input2.time(1,end)+eps])
            if i == n && isfield(input1,'name') && isfield(input2,'name') 
                legend(input1.name,input2.name,'Location','SouthEast')
            end
        end
        if nargin >= 3
            plot(input3.time,input3.value(i,:),'r');
            xlim([input3.time(1,1),input3.time(1,end)+eps])
            if i == n && isfield(input1,'name') && isfield(input2,'name') && isfield(input3,'name') 
                legend(input1.name,input2.name,input3.name,'Location','SouthEast')
            end
        end
            
        if isfield(input1, 'sigma') && isfield(input1, 'mean')
            if nargin >= 1
                if size(input1.sigma,1) >= i
                    plot(input1.time,input1.mean(i,:)+3*input1.sigma(i,:),'b--');
                    plot(input1.time,input1.mean(i,:)-3*input1.sigma(i,:),'b--');
                end
            end
            if nargin >= 2
                if size(input2.sigma,1) >= i
                    plot(input2.time,input2.mean(i,:)+3*input2.sigma(i,:),'g--');
                    plot(input2.time,input2.mean(i,:)-3*input2.sigma(i,:),'g--');
                end
            end
            if nargin >= 3
                if size(input3.sigma,1) >= i
                    plot(input3.time,input3.mean(i,:)+3*input3.sigma(i,:),'r--');
                    plot(input3.time,input3.mean(i,:)-3*input3.sigma(i,:),'r--');
                end
            end
        elseif isfield(input1,'sigma')
            if nargin >= 1
                if size(input1.sigma,1) >= i
                    plot(input1.time,input1.value(i,:)+3*input1.sigma(i,:),'b--');
                    plot(input1.time,input1.value(i,:)-3*input1.sigma(i,:),'b--');
                end
            end
            if nargin >= 2
                if size(input2.sigma,1) >= i
                    plot(input2.time,input2.value(i,:)+3*input2.sigma(i,:),'g--');
                    plot(input2.time,input2.value(i,:)-3*input2.sigma(i,:),'g--');
                end
            end
            if nargin >= 3
                if size(input3.sigma,1) >= i
                    plot(input3.time,input3.value(i,:)+3*input3.sigma(i,:),'r--');
                    plot(input3.time,input3.value(i,:)-3*input3.sigma(i,:),'r--');
                end
            end
        end
        if isfield(input1,'axis1') && i == 1
            lab = ylabel(input1.axis1); set(lab, 'interpreter', 'latex', 'FontSize', 14);
        end
        if isfield(input1,'axis2') && i == 2
            lab = ylabel(input1.axis2); set(lab, 'interpreter', 'latex', 'FontSize', 14);
        end
        if isfield(input1,'axis3') && i == 3
            lab = ylabel(input1.axis3); set(lab, 'interpreter', 'latex', 'FontSize', 14);
        end
        if isfield(input1,'axis4') && i == 4
            lab = ylabel(input1.axis4); set(lab, 'interpreter', 'latex', 'FontSize', 14);
        end
        if isfield(input1,'axis5') && i == 5
            lab = ylabel(input1.axis5); set(lab, 'interpreter', 'latex', 'FontSize', 14);
        end
        if isfield(input1,'base') && i == n
            lab = xlabel(input1.base); set(lab, 'interpreter', 'latex', 'FontSize', 14);
        end
    end
    
    linkaxes(axs, 'x');
    
    if isfield(input1, 'pdffile')
        print(h, '-dpdf', input1.pdffile);
    end
    
end

