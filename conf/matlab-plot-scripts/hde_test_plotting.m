close all;
clear all;
clc;

%% Plot parameters
fontSize = 20;
lineWidth = 2;


%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

subjectWeight = 75.5 * 9.81;

%% Measurement wrenches in link frame
LeftFootMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';
LeftHandMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(13:18,:)';
RightHandMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(19:24,:)';

normLeftFootMeasuredWrenchInLinkFrame = [];
normRightFootMeasuredWrenchInLinkFrame = [];

for i=1:size(LeftFootMeasuredWrenchInLinkFrame, 1)
    normLeftFootMeasuredWrenchInLinkFrame(i) = norm(LeftFootMeasuredWrenchInLinkFrame(i,1:3));
end

for i=1:size(RightFootMeasuredWrenchInLinkFrame, 1)
    normRightFootMeasuredWrenchInLinkFrame(i) = norm(RightFootMeasuredWrenchInLinkFrame(i,1:3));
end

%% Measurement wrenches in base frame
LeftFootMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(1:6,:)';
RightFootMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(7:12,:)';
LeftHandMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(13:18,:)';
RightHandMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(19:24,:)';

sumWrenchMeasurementsInBaseFrame = LeftFootMeasuredWrenchInBaseFrame + RightFootMeasuredWrenchInBaseFrame +...
                                   LeftHandMeasuredWrenchInBaseFrame + RightHandMeasuredWrenchInBaseFrame;
                               
sumFeetWrenchMeasurementsInBaseFrame = LeftFootMeasuredWrenchInBaseFrame + RightFootMeasuredWrenchInBaseFrame;

feetForceMeasurementsNormInBaseFrame = [];

for i=1:size(sumFeetWrenchMeasurementsInBaseFrame, 1)
    feetForceMeasurementsNormInBaseFrame(i) = norm(sumFeetWrenchMeasurementsInBaseFrame(i,1:3));
end

sumHandWrenchMeasurementsInBaseFrame = LeftHandMeasuredWrenchInBaseFrame + RightHandMeasuredWrenchInBaseFrame;

%% Measurement wrenches in world frame
LeftFootMeasuredWrenchInWorldFrame  = data.task1_wrenchMeasurementsInWorldFrame(1:6,:)';
RightFootMeasuredWrenchInWorldFrame = data.task1_wrenchMeasurementsInWorldFrame(7:12,:)';
LeftHandMeasuredWrenchInWorldFrame  = data.task1_wrenchMeasurementsInWorldFrame(13:18,:)';
RightHandMeasuredWrenchInWorldFrame = data.task1_wrenchMeasurementsInWorldFrame(19:24,:)';

sumWrenchMeasurementsInWorldFrame = LeftFootMeasuredWrenchInWorldFrame + RightFootMeasuredWrenchInWorldFrame +...
                                    LeftHandMeasuredWrenchInWorldFrame + RightHandMeasuredWrenchInWorldFrame;
                                
sumFeetWrenchMeasurementsInWorldFrame = LeftFootMeasuredWrenchInWorldFrame + RightFootMeasuredWrenchInWorldFrame;

feetForceMeasurementsNormInWorldFrame = [];

for i=1:size(sumFeetWrenchMeasurementsInWorldFrame, 1)
    feetForceMeasurementsNormInWorldFrame(i) = norm(sumFeetWrenchMeasurementsInWorldFrame(i,1:3));
end

sumHandWrenchMeasurementsInWorldFrame = LeftHandMeasuredWrenchInWorldFrame + RightHandMeasuredWrenchInWorldFrame;

%% Measurement wrenches in centroidal frame
LeftFootMeasuredWrenchInCentroidalFrame  = data.task1_wrenchMeasurementsInCentroidalFrame(1:6,:)';
RightFootMeasuredWrenchInCentroidalFrame = data.task1_wrenchMeasurementsInCentroidalFrame(7:12,:)';
LeftHandMeasuredWrenchInCentroidalFrame  = data.task1_wrenchMeasurementsInCentroidalFrame(13:18,:)';
RightHandMeasuredWrenchInCentroidalFrame = data.task1_wrenchMeasurementsInCentroidalFrame(19:24,:)';

sumWrenchMeasurementsInCentroidalFrame = LeftFootMeasuredWrenchInCentroidalFrame + RightFootMeasuredWrenchInCentroidalFrame +...
                                         LeftHandMeasuredWrenchInCentroidalFrame + RightHandMeasuredWrenchInCentroidalFrame;

%% Estimate wrenches in link frame
LeftFootEstimatedWrenchInLinkFrame  = data.task1_wrenchEstimatesInLinkFrame(1:6,:)';
RightFootEstimatedWrenchInLinkFrame = data.task1_wrenchEstimatesInLinkFrame(7:12,:)';
LeftHandEstimatedWrenchInLinkFrame  = data.task1_wrenchEstimatesInLinkFrame(13:18,:)';
RightHandEstimatedWrenchInLinkFrame = data.task1_wrenchEstimatesInLinkFrame(19:24,:)';

%% Estimate wrenches in base frame
LeftFootEstimatedWrenchInBaseFrame  = data.task1_wrenchEstimatesInBaseFrame(1:6,:)';
RightFootEstimatedWrenchInBaseFrame = data.task1_wrenchEstimatesInBaseFrame(7:12,:)';
LeftHandEstimatedWrenchInBaseFrame  = data.task1_wrenchEstimatesInBaseFrame(13:18,:)';
RightHandEstimatedWrenchInBaseFrame = data.task1_wrenchEstimatesInBaseFrame(19:24,:)';

sumWrenchEstimatesInBaseFrame = LeftFootEstimatedWrenchInBaseFrame + RightFootEstimatedWrenchInBaseFrame +...
                                LeftHandEstimatedWrenchInBaseFrame + RightHandEstimatedWrenchInBaseFrame;
                            
sumFeetWrenchEstimatesInBaseFrame = LeftFootEstimatedWrenchInBaseFrame + RightFootEstimatedWrenchInBaseFrame;
                            
sumHandWrenchEstimatesInBaseFrame = LeftHandEstimatedWrenchInBaseFrame + RightHandEstimatedWrenchInBaseFrame;

%% Estimate wrenches in world frame
LeftFootEstimatedWrenchInWorldFrame  = data.task1_wrenchEstimatesInWorldFrame(1:6,:)';
RightFootEstimatedWrenchInWorldFrame = data.task1_wrenchEstimatesInWorldFrame(7:12,:)';
LeftHandEstimatedWrenchInWorldFrame  = data.task1_wrenchEstimatesInWorldFrame(13:18,:)';
RightHandEstimatedWrenchInWorldFrame = data.task1_wrenchEstimatesInWorldFrame(19:24,:)';

sumWrenchEstimatesInWorldFrame = LeftFootEstimatedWrenchInWorldFrame + RightFootEstimatedWrenchInWorldFrame +...
                                 LeftHandEstimatedWrenchInWorldFrame + RightHandEstimatedWrenchInWorldFrame;
                             
sumFeetWrenchEstimatesInWorldFrame = LeftFootEstimatedWrenchInWorldFrame + RightFootEstimatedWrenchInWorldFrame;

sumHandWrenchEstimatesInWorldFrame = LeftHandEstimatedWrenchInWorldFrame + RightHandEstimatedWrenchInWorldFrame;


%% Estimate wrenches in centroidal frame
LeftFootEstimatedWrenchInCentroidalFrame  = data.task1_wrenchEstimatesInCentroidalFrame(1:6,:)';
RightFootEstimatedWrenchInCentroidalFrame = data.task1_wrenchEstimatesInCentroidalFrame(7:12,:)';
LeftHandEstimatedWrenchInCentroidalFrame  = data.task1_wrenchEstimatesInCentroidalFrame(13:18,:)';
RightHandEstimatedWrenchInCentroidalFrame = data.task1_wrenchEstimatesInCentroidalFrame(19:24,:)';

sumWrenchEstimatesInCentroidalFrame = LeftFootEstimatedWrenchInCentroidalFrame + RightFootEstimatedWrenchInCentroidalFrame +...
                                      LeftHandEstimatedWrenchInCentroidalFrame + RightHandEstimatedWrenchInCentroidalFrame;


%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
wrenchEstimatesLegendString = ["$\hat{f}_x [N]$", "$\hat{f}_y [N]$", "$\hat{f}_z [N]$","$\hat{m}_x [Nm]$", "$\hat{m}_y [Nm]$", "$\hat{m}_z [Nm]$"];
wrenchSourceName = ["Left Foot Wrench", "Right Foot Wrench", "Left Hand Wrench", "Right Hand Wrench"];
momentumLegendString = ["$^{B}\underline{\dot{h}}_{L_x}$", "$^{B}\underline{\dot{h}}_{L_y}$", "$^{B}\underline{\dot{h}}_{L_z}$",...
                        "$^{B}\underline{\dot{h}}_{\omega_x}$", "$^{B}\underline{\dot{h}}_{\omega_y}$", "$^{B}\underline{\dot{h}}_{\omega_z}$"];

%% Measurement Vs Estimates Wrench In Base Frame

numberOfWrenchSources = 4;

for i = 1:numberOfWrenchSources
    
    fH = figure('units','normalized','outerposition',[0 0 1 1]);
    
    for s = 1:6
        
        subplot(2,3,s);
        plot(data.task1_wrenchMeasurementsInLinkFrame(s + 6 * (i-1),:)', 'LineWidth', lineWidth);
        hold on;
        plot(data.task1_wrenchEstimatesInLinkFrame(s + 6 * (i-1),:)', 'LineWidth', lineWidth, 'LineStyle', '--');
        hold on;
%         ylim([-400 800])
        xlabel('Samples', 'FontSize', fontSize);
        ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
        set (gca, 'FontSize' , fontSize)
        legend('Measured Wrench', 'Estimated Wrench', 'FontSize', fontSize, 'Location', 'Best');
        
    end
    
    a = axes;
    t = title (strcat(wrenchSourceName(i) + " In Link Frame"));
    t.FontSize = fontSize;
    a.Visible = 'off' ;
    t.Visible = 'on' ;
    
    %% Save figure
    save2pdf(strcat(t.String + ".pdf"), fH,300);
    
end

fH = figure('units','normalized','outerposition',[0 0 1 1]);
    
for s = 1:6
        
    subplot(2,3,s);
    plot(LeftFootMeasuredWrenchInBaseFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(RightFootMeasuredWrenchInBaseFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(sumFeetWrenchMeasurementsInBaseFrame(:,s), 'LineWidth', lineWidth);
    hold on;
%     ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Left Foot', 'Right Foot', 'Left + Right', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Feet wrench measurements expressed in base frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
    
% % for s = 1:6
% %         
% %     subplot(2,3,s);
% %     plot(sumHandWrenchMeasurementsInWorldFrame(:,s), 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumHandWrenchEstimatesInWorldFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;
% %     ylim([-400 800])
% %     xlabel('Samples', 'FontSize', fontSize);
% %     ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
% %     set (gca, 'FontSize' , fontSize)
% %     legend('Measured Wrench', 'Estimated Wrench', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %         
% % end
% % 
% % a = axes;
% % t = title ("Sum of hands wrench measurements and estimates expressed in world frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % 
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% %     
% % for s = 1:6
% %         
% %     subplot(2,3,s);
% %     plot(sumFeetWrenchMeasurementsInWorldFrame(:,s), 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumFeetWrenchEstimatesInWorldFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;
% %     ylim([-800 1200])
% %     xlabel('Samples', 'FontSize', fontSize);
% %     ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
% %     set (gca, 'FontSize' , fontSize)
% %     legend('Measured Wrench', 'Estimated Wrench', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %         
% % end
% % 
% % a = axes;
% % t = title ("Sum of feet wrench measurements expressed in world frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % plot(feetForceMeasurementsNormInWorldFrame, 'LineWidth', lineWidth);
% % hold on;
% % yline(subjectWeight, 'LineWidth', lineWidth, 'LineStyle', '--');
% % hold on;
% % ylim([600 900])
% % xlabel('Samples', 'FontSize', fontSize);
% % ylabel('$Weight$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % legend('Measured Weight', 'Subject Weight', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% % 
% % a = axes;
% % t = title ("Norm of sum of feet force measurements expressed in world frame Vs subject weight");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;

% % %% Feet measured force norm expressed in link frames
% % 
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % subplot(3,1,1)
% % plot(normLeftFootMeasuredWrenchInLinkFrame', 'LineWidth', lineWidth);
% % hold on;
% % yline(subjectWeight/2, 'LineWidth', lineWidth, 'LineStyle', '--');
% % hold on;
% % 
% % ylim([200 1000])
% % xlabel('Samples', 'FontSize', fontSize);
% % ylabel('$Weight (Left Foot )$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % legend('Measured Weight $|| {}^{\mathcal{LF}}{f}_{LF} ||$', '$\frac{Subject Weight}{2}$', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% % 
% % subplot(3,1,2)
% % plot(normRightFootMeasuredWrenchInLinkFrame', 'LineWidth', lineWidth);
% % hold on;
% % yline(subjectWeight/2, 'LineWidth', lineWidth, 'LineStyle', '--');
% % hold on;
% % 
% % ylim([200 1000])
% % xlabel('Samples', 'FontSize', fontSize);
% % ylabel('$Weight (Right Foot)$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % legend('Measured Weight $|| {}^{\mathcal{RF}}{f}_{RF} ||$', '$\frac{Subject Weight}{2}$', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% % 
% % subplot(3,1,3)
% % plot(normLeftFootMeasuredWrenchInLinkFrame'+normRightFootMeasuredWrenchInLinkFrame', 'LineWidth', lineWidth);
% % hold on;
% % yline(subjectWeight, 'LineWidth', lineWidth, 'LineStyle', '--');
% % hold on;
% % 
% % 
% % ylim([200 1000])
% % xlabel('Samples', 'FontSize', fontSize);
% % ylabel('$Weight (Total)$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % legend('Measured Weight $|| {}^{\mathcal{LF}}{f}_{LF} || + || {}^{\mathcal{RF}}{f}_{RF} ||$', 'Subject Weight', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% % 
% % a = axes;
% % t = title ("Norm of feet force measurements expressed in link frame Vs subject weight");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;

% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % subplot(2,1,1)
% % plot(feetForceMeasurementsNormInWorldFrame, 'LineWidth', lineWidth);
% % hold on;
% % plot(normLeftFootMeasuredWrenchInLinkFrame'+normRightFootMeasuredWrenchInLinkFrame', 'LineWidth', lineWidth);
% % hold on;
% % yline(subjectWeight, 'LineWidth', lineWidth, 'LineStyle', '--');
% % hold on;
% % ylim([600 900])
% % xlabel('Samples', 'FontSize', fontSize);
% % ylabel('$Weight$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % legend('Measured Weight $|| {}^{\mathcal{I}}{f}_{LF} + {}^{\mathcal{I}}{f}_{RF} ||$', 'Measured Weight $|| {}^{\mathcal{LF}}{f}_{LF} || + || {}^{\mathcal{RF}}{f}_{RF} ||$', 'Subject Weight', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% % 
% % subplot(2,1,2)
% % plot(feetForceMeasurementsNormInWorldFrame - (normLeftFootMeasuredWrenchInLinkFrame'+normRightFootMeasuredWrenchInLinkFrame')', 'LineWidth', lineWidth);
% % hold on;
% % xlabel('Samples', 'FontSize', fontSize);
% % ylabel('$Weight$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % legend('$|| {}^{\mathcal{I}}{f}_{LF} + {}^{\mathcal{I}}{f}_{RF} || - \Big( || {}^{\mathcal{LF}}{f}_{LF} || + || {}^{\mathcal{RF}}{f}_{RF} || \Big)$', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% % 
% % 
% % a = axes;
% % t = title ("Norm of sum of feet force measurements expressed in world frame Vs Sum of norm of feet force measurements expressed in link frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;


% % %% Sum of measured wrench vs sum of estimated wrench in Base frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6
% %         
% %         subplot(2,3,s);
% %         plot(sumWrenchMeasurementsInBaseFrame(:,s), 'LineWidth', lineWidth);
% %         hold on;
% %         plot(sumWrenchEstimatesInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %         hold on;
% %         xlabel('Samples', 'FontSize', fontSize);
% %         ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
% %         set (gca, 'FontSize' , fontSize)
% %         legend('Measured Wrench', 'Estimated Wrench', 'FontSize', fontSize, 'Location', 'Best');
% %         
% % end
% % 
% % a = axes;
% % t = title ("Sum of measured wrench vs sum of estimated wrench in Base frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % %% Save figure
% % save2pdf("SumOfMeasuredVsEstimatedWrenchInBaseFrame.pdf", fH,300);

%% Rate of change of momentum
RateOfChangeOfMomentumInWorldFrame = data.rateOfChangeOfMomentumInWorldFrame;
RateOfChangeOfMomentumInBaseFrame = data.rateOfChangeOfMomentumInBaseFrame;
RateOfChangeOfMomentumInCentroidalFrame = data.rateOfChangeOfMomentumInCentroidalFrame;

%%   Rate of Change of Momentum In Base Frame Vs Sum of External Wrenches Measurements In Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6
    
    subplot(2,3,s);
    plot( RateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumWrenchMeasurementsInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;
    plot(sumWrenchEstimatesInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '-.');
    hold on;
%     ylim([-400 1000])
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), wrenchEstimatesLegendString(s),...
           'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Rate of Change of Momentum - Sum of External Wrenches Measurements In Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

% % save2pdf("rateOfMomentumVsMeasuredWrenchesInBaseFrame.pdf", fH,300);


% % %% %% Center of Mass Position Expressed in World Frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % plot( data.comPosition', 'LineWidth', lineWidth);
% % hold on;
% % xlabel('Samples', 'FontSize', fontSize);
% % legend('$com_x$','$com_y$','$com_z$', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% % 
% % 
% % a = axes;
% % t = title ("Center of Mass Position in World Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % save2pdf("comPositionInWorldFrame.pdf", fH,300);


% % %% Feet measurements in Link and Base Frames
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6
% %     
% %     subplot(2,3,s);
% %     plot( LeftFootMeasuredWrenchInLinkFrame(:,s), 'LineWidth', lineWidth);
% %     hold on;
% %     plot(LeftFootMeasuredWrenchInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(strcat(wrenchLegendString(s), " link frame"), strcat(wrenchLegendString(s), " base frame"),...
% %           'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("Left Foot Measured Wrench in Link Frame Vs Base Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % 
% % 
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6
% %     
% %     subplot(2,3,s);
% %     plot( RightFootMeasuredWrenchInLinkFrame(:,s), 'LineWidth', lineWidth);
% %     hold on;
% %     plot(RightFootMeasuredWrenchInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(strcat(wrenchLegendString(s), " link frame"), strcat(wrenchLegendString(s), " base frame"),...
% %           'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("Right Foot Measured Wrench in Link Frame Vs Base Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % 
% % 
% % 
% % 


























% % %% %%   Rate of Change of Momentum In Base Frame Vs Sum of External Wrenches Measurements In Base Frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6 
% %     
% %     subplot(2,3,s);
% %     plot( RateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumWrenchMeasurementsInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;    
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("  Rate of Change of Momentum - Sum of External Wrenches Measurements In Base Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % save2pdf("rateOfMomentumVsMeasuredWrenchesInBaseFrame.pdf", fH,300);
% % 
% % %%   Rate of Change of Momentum In World Frame Vs Sum of External Wrenches Measurements In World Frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6 
% %     
% %     subplot(2,3,s);
% %     plot( RateOfChangeOfMomentumInWorldFrame(s,:)', 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumWrenchMeasurementsInWorldFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(momentumLegendString(s), wrenchLegendString(s), strcat("offset-"+wrenchLegendString(s)), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("  Rate of Change of Momentum - Sum of External Wrenches Measurements In World Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % save2pdf("rateOfMomentumVsMeasuredWrenchesInWorldFrame.pdf", fH,300);

% % %% %%   Rate of Change of Momentum In World Frame Vs Sum of External Estimated Wrenches In Base Frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6 
% %     
% %     subplot(2,3,s);
% %     plot( RateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumOfEstimatedWrenchInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;    
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("  Rate of Change of Momentum - Sum of External Estimated Wrenches In Base Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % save2pdf("rateOfMomentumVsEstimatedWrenchesInBaseFrame.pdf", fH,300);

% % %% %%   Rate of Change of Momentum In World Frame Vs Sum of External Estimated Wrenches In World Frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6 
% %     
% %     subplot(2,3,s);
% %     plot( RateOfChangeOfMomentumInWorldFrame(s,:)', 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumOfEstimatedWrenchInWorldFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;    
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("  Rate of Change of Momentum - Sum of External Estimated Wrenches In World Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % save2pdf("rateOfMomentumVsEstimatedWrenchesInWorldFrame.pdf", fH,300);



