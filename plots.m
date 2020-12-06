% Script to plot the results of the simulation:

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

if simType.integrationAnalysis
    
end