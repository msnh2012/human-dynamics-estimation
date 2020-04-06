close all;
clear all;
clc;

%% Plot parameters
fontSize = 20;
lineWidth = 2;

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

%% Measurement wrenches in link frame
LeftFootMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';
LeftHandMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(13:18,:)';
RightHandMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(19:24,:)';

%% Measurement wrenches in base frame
LeftFootMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(1:6,:)';
RightFootMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(7:12,:)';
LeftHandMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(13:18,:)';
RightHandMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(19:24,:)';

sumWrenchMeasurementsInBaseFrame = LeftFootMeasuredWrenchInBaseFrame + RightFootMeasuredWrenchInBaseFrame +...
                                   LeftHandMeasuredWrenchInBaseFrame + RightHandMeasuredWrenchInBaseFrame;

%% Measurement wrenches in world frame
LeftFootMeasuredWrenchInWorldFrame  = data.task1_wrenchMeasurementsInWorldFrame(1:6,:)';
RightFootMeasuredWrenchInWorldFrame = data.task1_wrenchMeasurementsInWorldFrame(7:12,:)';
LeftHandMeasuredWrenchInWorldFrame  = data.task1_wrenchMeasurementsInWorldFrame(13:18,:)';
RightHandMeasuredWrenchInWorldFrame = data.task1_wrenchMeasurementsInWorldFrame(19:24,:)';

sumWrenchMeasurementsInWorldFrame = LeftFootMeasuredWrenchInWorldFrame + RightFootMeasuredWrenchInWorldFrame +...
                                    LeftHandMeasuredWrenchInWorldFrame + RightHandMeasuredWrenchInWorldFrame;

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

%% Estimate wrenches in world frame
LeftFootEstimatedWrenchInWorldFrame  = data.task1_wrenchEstimatesInWorldFrame(1:6,:)';
RightFootEstimatedWrenchInWorldFrame = data.task1_wrenchEstimatesInWorldFrame(7:12,:)';
LeftHandEstimatedWrenchInWorldFrame  = data.task1_wrenchEstimatesInWorldFrame(13:18,:)';
RightHandEstimatedWrenchInWorldFrame = data.task1_wrenchEstimatesInWorldFrame(19:24,:)';

sumWrenchEstimatesInWorldFrame = LeftFootEstimatedWrenchInWorldFrame + RightFootEstimatedWrenchInWorldFrame +...
                                 LeftHandEstimatedWrenchInWorldFrame + RightHandEstimatedWrenchInWorldFrame;

%% Estimate wrenches in centroidal frame
LeftFootEstimatedWrenchInCentroidalFrame  = data.task1_wrenchEstimatesInCentroidalFrame(1:6,:)';
RightFootEstimatedWrenchInCentroidalFrame = data.task1_wrenchEstimatesInCentroidalFrame(7:12,:)';
LeftHandEstimatedWrenchInCentroidalFrame  = data.task1_wrenchEstimatesInCentroidalFrame(13:18,:)';
RightHandEstimatedWrenchInCentroidalFrame = data.task1_wrenchEstimatesInCentroidalFrame(19:24,:)';

sumWrenchEstimatesInCentroidalFrame = LeftFootEstimatedWrenchInCentroidalFrame + RightFootEstimatedWrenchInCentroidalFrame +...
                                      LeftHandEstimatedWrenchInCentroidalFrame + RightHandEstimatedWrenchInCentroidalFrame;


%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
wrenchSourceName = ["Left Foot Wrench", "Right Foot Wrench", "Left Hand Wrench", "Right Hand Wrench"];
momentumLegendString = ["$\dot{H}_{L_x}$", "$\dot{H}_{L_y}$", "$\dot{H}_{L_z}$", "$\dot{H}_{\omega_x}$", "$\dot{H}_{\omega_y}$", "$\dot{H}_{\omega_z}$"];

%% Measurement Vs Estimates Wrench In Base Frame

numberOfWrenchSources = 4;

for i = 1:numberOfWrenchSources
    
    fH = figure('units','normalized','outerposition',[0 0 1 1]);
    
    for s = 1:6
        
        subplot(2,3,s);
        plot(data.task1_wrenchMeasurementsInBaseFrame(s + 6 * (i-1),:)', 'LineWidth', lineWidth);
        hold on;
        plot(data.task1_wrenchEstimatesInBaseFrame(s + 6 * (i-1),:)', 'LineWidth', lineWidth, 'LineStyle', '--');
        hold on;
        xlabel('Samples', 'FontSize', fontSize);
        ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
        set (gca, 'FontSize' , fontSize)
        legend('Measured Wrench', 'Estimated Wrench', 'FontSize', fontSize, 'Location', 'Best');
        
    end
    
    a = axes;
    t = title (strcat(wrenchSourceName(i) + " In Base Frame"));
    t.FontSize = fontSize;
    a.Visible = 'off' ;
    t.Visible = 'on' ;
    
    %% Save figure
    save2pdf(strcat(t.String + ".pdf"), fH,300);
    
end

%% Sum of measured wrench vs sum of estimated wrench in Base frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6
        
        subplot(2,3,s);
        plot(sumWrenchMeasurementsInBaseFrame(:,s), 'LineWidth', lineWidth);
        hold on;
        plot(sumWrenchEstimatesInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
        hold on;
        xlabel('Samples', 'FontSize', fontSize);
        ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
        set (gca, 'FontSize' , fontSize)
        legend('Measured Wrench', 'Estimated Wrench', 'FontSize', fontSize, 'Location', 'Best');
        
end

a = axes;
t = title ("Sum of measured wrench vs sum of estimated wrench in Base frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

%% Save figure
save2pdf("SumOfMeasuredVsEstimatedWrenchInBaseFrame.pdf", fH,300);

% % %% Rate of change of momentum
% % RateOfChangeOfMomentumInWorldFrame = data.rateOfChangeOfMomentumInWorldFrame;
% % RateOfChangeOfMomentumInBaseFrame = data.rateOfChangeOfMomentumInBaseFrame;
% % RateOfChangeOfMomentumInCentroidalFrame = data.rateOfChangeOfMomentumInCentroidalFrame;
% % 
% % %% %%   Rate of Change of Momentum In Centroidal Frame Vs Sum of External Wrenches Measurements In Centroidal Frame
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % for s = 1:6
% %     
% %     subplot(2,3,s);
% %     plot( RateOfChangeOfMomentumInCentroidalFrame(s,:)', 'LineWidth', lineWidth);
% %     hold on;
% %     plot(sumWrenchMeasurementsInCentroidalFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
% %     hold on;
% %     xlabel('Samples', 'FontSize', fontSize);
% %     legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
% %     
% % end
% % 
% % a = axes;
% % t = title ("  Rate of Change of Momentum - Sum of External Wrenches Measurements In Centroidal Frame");
% % t.FontSize = fontSize;
% % a.Visible = 'off' ;
% % t.Visible = 'on' ;
% % 
% % save2pdf("rateOfMomentumVsMeasuredWrenchesInCentroidalFrame.pdf", fH,300);


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



