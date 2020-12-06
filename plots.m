% Script to plot the results of the simulation:

if simType.main
    
    % Semi major axis:
    figure,
    plot(T,Y(:,7),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Semi Major Axis [km]');
    grid on, box on
   %title('Nominal Response - Semi Major Axis')
    saveFigAsPdf('6-SMAxis',0.7)
    
    % FCV:
    figure,
    subplot(2,1,1)
    plot(T,Y(:,2),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Spool Position [m]')
    grid on, box on
    subplot(2,1,2)
    plot(T,Y(:,3),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Spool Velocity [m/s]')
    grid on, box on
   %suptitle('Nominal Response - Flow Control Valve')
    saveFigAsPdf('6-FCV',0.7)
    
    % Accelerometer:
    figure,
    subplot(2,1,1)
    plot(T,Y(:,4),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Mass Position [m]')
    grid on, box on
    subplot(2,1,2)
    plot(T,Y(:,5),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Mass Velocity [m/s]')
    grid on, box on
   %suptitle('Nominal Response - Accelerometer')
    saveFigAsPdf('6-Accelerometer',0.7)
    
    % Output Voltage:
    figure,
    plot(T,Y(:,6),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('$V_{out}$ [V]')
    grid on, box on
   %title('Nominal Response - Output voltage of read-out circuit')
    saveFigAsPdf('6-VOut',0.7)
    
    % Res. Acceleration:
    figure,
    plot(T,out.residualAcc,'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Res. Acceleration $[m/s^2]$')
    grid on, box on
   %title('Nominal Response - Res. Acceleration')
    saveFigAsPdf('6-residualAcc',0.7)
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
    xlabel('Time $[s]$'), ylabel('Res. Acceleration $[m/s^2]$')
    saveFigAsPdf('7-envelope',0.7)
end

if simType.failures
    
    % Res. Acceleration:
    figure,
    plot(TF,outF.residualAcc,'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Res. Acceleration $[m/s^2]$')
    grid on, box on
   %title('Failures - Res. Acceleration')
    h1 = line([dataF.noThrust.tInitial dataF.noThrust.tInitial],get(gca,'YLim'),'color','r',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.noThrust.tFinal dataF.noThrust.tFinal],get(gca,'YLim'),'color','r',...
        'linewidth',1.5,'linestyle','--')
    h2 = line([dataF.blockFCV.tInitial dataF.blockFCV.tInitial],get(gca,'YLim'),'color','b',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.blockFCV.tFinal dataF.blockFCV.tFinal],get(gca,'YLim'),'color','b',...
        'linewidth',1.5,'linestyle','--')
    legend([h1;h2],'Thruster failure','Valve failure')
    saveFigAsPdf('6-Failures-residualAcc',0.7)
    
    % FCV:
    figure,
    subplot(2,1,1)
    plot(TF,YF(:,2),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Spool Position [m]')
    grid on, box on
    line([dataF.noThrust.tInitial dataF.noThrust.tInitial],get(gca,'YLim'),'color','r',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.noThrust.tFinal dataF.noThrust.tFinal],get(gca,'YLim'),'color','r',...
        'linewidth',1.5,'linestyle','--')
    line([dataF.blockFCV.tInitial dataF.blockFCV.tInitial],get(gca,'YLim'),'color','b',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.blockFCV.tFinal dataF.blockFCV.tFinal],get(gca,'YLim'),'color','b',...
        'linewidth',1.5,'linestyle','--')
    subplot(2,1,2)
    plot(TF,YF(:,3),'k','linewidth',1.5)
    xlabel('Time [s]'), ylabel('Spool Velocity [m/s]')
    ylim([-4 6]*1e-10)
    grid on, box on
   %suptitle('Failures - Flow Control Valve')
    h1 = line([dataF.noThrust.tInitial dataF.noThrust.tInitial],get(gca,'YLim'),'color','r',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.noThrust.tFinal dataF.noThrust.tFinal],get(gca,'YLim'),'color','r',...
        'linewidth',1.5,'linestyle','--')
    h2 = line([dataF.blockFCV.tInitial dataF.blockFCV.tInitial],get(gca,'YLim'),'color','b',...
        'linewidth',1.5,'linestyle','--');
    line([dataF.blockFCV.tFinal dataF.blockFCV.tFinal],get(gca,'YLim'),'color','b',...
        'linewidth',1.5,'linestyle','--')
        legend([h1;h2],'Thruster failure','Valve failure')
    saveFigAsPdf('6-Failures-FCV',0.7)
end