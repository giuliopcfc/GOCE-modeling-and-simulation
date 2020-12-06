% Script to plot the results of the simulation:
%% Main Simulation:
if simType.main
    
    % Semi major axis:
    figure,
    plot(T,Y(:,7),'k','linewidth',1.5)
    xlabel('$t [s]$'), ylabel('$a [km]$');
    grid on, box on
    %title('Nominal Response - Semi Major Axis')
    saveFigAsPdf('6-SMAxis',0.49,2.5)
    
    % FCV:
    figure,
    subplot(2,1,1)
    plot(T,Y(:,2),'k','linewidth',1.5)
    ylabel('$x_v [m]$')
    grid on, box on
    subplot(2,1,2)
    plot(T,Y(:,3),'k','linewidth',1.5)
    xlabel('$t [s]$'), ylabel('$v_v [m/s]$')
    grid on, box on
    %suptitle('Nominal Response - Flow Control Valve')
    saveFigAsPdf('6-FCV',0.49)
    
    % Accelerometer:
    figure,
    subplot(2,1,1)
    plot(T,Y(:,4),'k','linewidth',1.5)
    ylabel('$x_a [m]$')
    grid on, box on
    subplot(2,1,2)
    plot(T,Y(:,5),'k','linewidth',1.5)
    xlabel('$t [s]$'), ylabel('$v_a [m/s]$')
    grid on, box on
    %suptitle('Nominal Response - Accelerometer')
    saveFigAsPdf('6-Accelerometer',0.49)
    
    % Output Voltage:
    figure,
    plot(T,Y(:,6),'k','linewidth',1.5)
    xlabel('$t [s]$'), ylabel('$V_{out} [V]$')
    grid on, box on
    %title('Nominal Response - Output voltage of read-out circuit')
    saveFigAsPdf('6-VOut',0.49,2.5)
    
    % Res. Acceleration:
    figure,
    plot(T,out.residualAcc,'k','linewidth',1.5)
    xlabel('$t [s]$'), ylabel('$a_{res} [m/s^2]$')
    grid on, box on
    %title('Nominal Response - Res. Acceleration')
    saveFigAsPdf('6-residualAcc',0.49,2.5)
end

%% Sensitivity Analysis
if simType.sensitivity
    
    figure,
    plot(tSA,meanResp,'r','linewidth',1.5)
    hold on
    plot(tSA,meanResp + stdResp,'--k','linewidth',1.5)
    hold on
    plot(tSA,meanResp - stdResp,'--k','linewidth',1.5)
    grid on, box on
    %title('System Response Envelope With Uncertainties')
    legend('Mean Response','Mean Response $ \pm\, \sigma (t)$')
    xlabel('$t [s]$'), ylabel('$a_{res} [m/s^2]$')
    saveFigAsPdf('6-sensitivity',0.49)
end

%% Failures:
if simType.failures
    
    % Res. Acceleration:
    figure,
    plot(TF,outF.residualAcc,'k','linewidth',1.5)
    ylim([min(outF.residualAcc) max(outF.residualAcc)]);
    xlabel('$t [s]$'), ylabel('$a_{res} [m/s^2]$')
    grid on, box on
    %title('Failures - Res. Acceleration')
    h1 = line([dataF.noThrust.tInitial dataF.noThrust.tInitial],1e5*[-1 1],'color','r',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.noThrust.tFinal dataF.noThrust.tFinal],1e5*[-1 1],'color','r',...
        'linewidth',1.5,'linestyle','--')
    h2 = line([dataF.blockFCV.tInitial dataF.blockFCV.tInitial],1e5*[-1 1],'color','b',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.blockFCV.tFinal dataF.blockFCV.tFinal],1e5*[-1 1],'color','b',...
        'linewidth',1.5,'linestyle','--')
    legend([h1;h2],'Thruster failure','Valve failure')
    saveFigAsPdf('6-failures-residualAcc',0.49,1.5)
    
    % FCV:
    figure,
    subplot(2,1,1)
    plot(TF,YF(:,2),'k','linewidth',1.5)
    ylim([min(YF(:,2)) max(YF(:,2))])
    ylabel('$x_v [m]$')
    grid on, box on
    h1 = line([dataF.noThrust.tInitial dataF.noThrust.tInitial],1e5*[-1 1],'color','r',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.noThrust.tFinal dataF.noThrust.tFinal],1e5*[-1 1],'color','r',...
        'linewidth',1.5,'linestyle','--')
    h2 = line([dataF.blockFCV.tInitial dataF.blockFCV.tInitial],1e5*[-1 1],'color','b',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.blockFCV.tFinal dataF.blockFCV.tFinal],1e5*[-1 1],'color','b',...
        'linewidth',1.5,'linestyle','--')
    legend([h1;h2],'Thruster failure','Valve failure')
    
    subplot(2,1,2)
    plot(TF,YF(:,3),'k','linewidth',1.5)
    xlabel('$t [s]$'), ylabel('$v_v [m/s]$')
    ylim([-4 6]*1e-10)
    grid on
    %suptitle('Failures - Flow Control Valve')
    line([dataF.noThrust.tInitial dataF.noThrust.tInitial],1e5*[-1 1],'color','r',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.noThrust.tFinal dataF.noThrust.tFinal],1e5*[-1 1],'color','r',...
        'linewidth',1.5,'linestyle','--')
    line([dataF.blockFCV.tInitial dataF.blockFCV.tInitial],1e5*[-1 1],'color','b',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.blockFCV.tFinal dataF.blockFCV.tFinal],1e5*[-1 1],'color','b',...
        'linewidth',1.5,'linestyle','--')
    saveFigAsPdf('6-failures-FCV',0.49,1.5)
end

%% Integration Analysis:
if simType.integrationAnalysis
    
    % CPU-time VS Tolerances
    figure,
    hold on
    legendAbsTol = cell(length(absTol),1);
    for i = 1:length(absTol)
        plot(relTol,cpuTimes(i,:),'linewidth',1.5);
        
        legendAbsTol{i} = ['$10^{',num2str(log10(absTol(i))),'}$'];
    end
    set(gca,'XScale','log')
    leg = legend(legendAbsTol,'interpreter','latex');
    title(leg,'\textbf{AbsTol}', 'Interpreter', 'Latex')
    grid on
    xlabel('RelTol'), ylabel('Time [s]')
    xlim([min(relTol) max(relTol)])
    saveFigAsPdf('4-comp-times',0.49)
    
    % Plot of the stability regions of BDFs along with 
    % eigenvalues from the linearization:
    x1 = [0 0]; y = [-8 8];
    
    n = 250;
    t = linspace(0,2*pi,n);
    z = exp(1i*t);
    
    figure,
    plot(8*y,x1, 'k', 'HandleVisibility', 'off')
    hold on
    plot(x1,8*y, 'k', 'HandleVisibility', 'off')
    
    d = 1-1./z; r = 0;
    
    for i = 1:5
        r = r+(d.^i)/i;
        plot(r, 'LineWidth', 2)
    end
    
    xlim([-10 20])
    xlabel('Re $\{ h \lambda \}$')
    ylim([-15 15])
    ylabel('Im $\{ h \lambda \}$')
    grid on
    legend('BDF1','BDF2','BDF3','BDF4','BDF5', 'Location', 'best')
    hold on
    
    colors = get(gca, 'ColorOrder');
    m = [-tand(86), -tand(73), -tand(51)]';
    x = [0 -1 -3 -5];
    p = plot(x(1:2), m(1)*x(1:2), '--', 'LineWidth', 1.5,...
        'HandleVisibility', 'off');
    p.Color = colors(5,:);
    hold on
    p = plot(x(1:2:3), m(2)*x(1:2:3), '--', 'LineWidth', 1.5,...
        'HandleVisibility', 'off');
    p.Color = colors(6,:);
    hold on
    p = plot(x(1:3:end), m(3)*x(1:3:end), '--', 'LineWidth', 1.5,...
        'HandleVisibility', 'off');
    p.Color = colors(7,:);
    saveFigAsPdf('4-bdf',0.49)
end