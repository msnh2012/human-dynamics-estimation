close all;
clear all;
clc;

%% Plot parameters
fontSize = 20;
lineWidth = 2;

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');
% plotSuffix = "HandEstimatesInLinkOrientation";
% plotSuffix = " - Expressed in Link Frame";=
% plotSuffix = " - Measurements in World Orientation & Estimates in Link Orientation";

%% Measurement wrenches in link frame
LeftFootMeasuredWrenchInLinkFrame = data.wrenchEstimates(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.wrenchEstimates(13:18,:)';
LeftHandMeasuredWrenchInLinkFrame = data.wrenchEstimates(25:30,:)';
RightHandMeasuredWrenchInLinkFrame = data.wrenchEstimates(37:42,:)';

%% Sum of measurements wrenches in base frame
sumMeasurementsInBaseFrame = data.wrenchEstimates(97:102,:)' + data.wrenchEstimates(109:114,:)' + data.wrenchEstimates(121:126,:)' + data.wrenchEstimates(133:138,:)';
sumFeetMeasurementsInBaseFrame = data.wrenchEstimates(97:102,:)' + data.wrenchEstimates(109:114,:)';
sumHandsMeasurementsInBaseFrame = data.wrenchEstimates(121:126,:)' + data.wrenchEstimates(133:138,:)';

sumMeasurementsInWorldFrame = data.wrenchEstimates(103:108,:)' + data.wrenchEstimates(115:120,:)' + data.wrenchEstimates(127:132,:)' + data.wrenchEstimates(139:144,:)';
sumFeetMeasurementsInWorldFrame = data.wrenchEstimates(103:108,:)' + data.wrenchEstimates(115:120,:)';
sumHandsMeasurementsInWorldFrame = data.wrenchEstimates(127:132,:)' + data.wrenchEstimates(139:144,:)';

%% Estimated wrenches in link frame
LeftFootEstimatedWrenchInLinkFrame = data.wrenchEstimates(7:12,:)';
RightFootEstimatedWrenchInLinkFrame = data.wrenchEstimates(19:24,:)';
LeftHandEstimatedWrenchInLinkFrame = data.wrenchEstimates(31:36,:)';
RightHandEstimatedWrenchInLinkFrame = data.wrenchEstimates(43:48,:)';

%% Sum of estimated wrenches in base frame
sumOfEstimatedWrenchInBaseFrame = data.wrenchEstimates(49:54,:)' + data.wrenchEstimates(61:66,:)' + data.wrenchEstimates(73:78,:)' + data.wrenchEstimates(85:90,:)';
sumOfFeetEstimatedWrenchInBaseFrame = data.wrenchEstimates(49:54,:)' + data.wrenchEstimates(61:66,:)';
sumOfHandsEstimatedWrenchInBaseFrame = data.wrenchEstimates(73:78,:)' + data.wrenchEstimates(85:90,:)';

%% Sum of estimated wrenches in world frame
sumOfEstimatedWrenchInWorldFrame = data.wrenchEstimates(55:60,:)' + data.wrenchEstimates(67:72,:)' + data.wrenchEstimates(79:84,:)' + data.wrenchEstimates(91:96,:)';
sumOfFeetEstimatedWrenchInWorldFrame = data.wrenchEstimates(55:60,:)' + data.wrenchEstimates(67:72,:)';
sumOfHandsEstimatedWrenchInWorldFrame = data.wrenchEstimates(79:84,:)' + data.wrenchEstimates(91:96,:)';

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
wrenchSourceName = {'Left Foot Wrench In Link Frame', 'Right Foot Wrench In Link Frame', 'Left Hand Wrench In Link Frame', 'Right Hand Wrench In Link Frame'};
momentumLegendString = ["$\dot{H}_{L_x}$", "$\dot{H}_{L_y}$", "$\dot{H}_{L_z}$", "$\dot{H}_{\omega_x}$", "$\dot{H}_{\omega_y}$", "$\dot{H}_{\omega_z}$"];

% % %% Measurement Vs Estimates Wrench In Link Frame
% % 
% % numberOfWrenchSources = 4;
% % 
% % for i = 1:numberOfWrenchSources
% %     
% %     fH = figure('units','normalized','outerposition',[0 0 1 1]);
% %     
% %     for s = 1:6
% %         
% %         subplot(2,3,s);
% %         plot(data.wrenchEstimates(s + 2 * 6 * (i-1),:)', 'LineWidth', lineWidth);
% %         hold on;
% %         plot(data.wrenchEstimates(6 + s + 2 * 6 * (i-1),:)', 'LineWidth', lineWidth, 'LineStyle', '--');
% %         hold on;
% %         xlabel('Samples', 'FontSize', fontSize);
% %         ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
% %         set (gca, 'FontSize' , fontSize)
% %         legend('Measured Wrench', 'Estimated Wrench', 'FontSize', fontSize, 'Location', 'Best');
% %         
% %     end
% %     
% %     a = axes;
% %     t = title (wrenchSourceName(i));
% %     t.FontSize = fontSize;
% %     a.Visible = 'off' ;
% %     t.Visible = 'on' ;
% %     
% %     %% Save figure
% % % %     save2pdf(strcat(wrenchSourceName(i) + ".pdf"), fH,300);
% %     
% % end

properRateOfChangeOfMomentumInWorldFrame = data.comProperAccelerationInWorldFrame;
properRateOfChangeOfMomentumInBaseFrame = data.comProperAccelerationInBaseFrame;

%% %% Proper Rate of Change of Momentum In Base Frame Vs Sum of External Feet Wrenches Measurements In Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6 
    
    subplot(2,3,s);
    plot(properRateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumMeasurementsInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;    
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Proper Rate of Change of Momentum - Sum of External Feet Wrenches Measurements In Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

save2pdf("rateOfMomentumVsMeasuredWrenchesInBaseFrame.pdf", fH,300);

%% %% Proper Rate of Change of Momentum In World Frame Vs Sum of External Feet Wrenches Measurements In World Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6 
    
    subplot(2,3,s);
    plot(properRateOfChangeOfMomentumInWorldFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumMeasurementsInWorldFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;    
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Proper Rate of Change of Momentum - Sum of External Feet Wrenches Measurements In World Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

save2pdf("rateOfMomentumVsMeasuredWrenchesInWorldFrame.pdf", fH,300);

%% %% Proper Rate of Change of Momentum In World Frame Vs Sum of External Estimated Wrenches In Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6 
    
    subplot(2,3,s);
    plot(properRateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumOfEstimatedWrenchInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;    
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Proper Rate of Change of Momentum - Sum of External Estimated Wrenches In Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

save2pdf("rateOfMomentumVsEstimatedWrenchesInBaseFrame.pdf", fH,300);

%% %% Proper Rate of Change of Momentum In World Frame Vs Sum of External Estimated Wrenches In World Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6 
    
    subplot(2,3,s);
    plot(properRateOfChangeOfMomentumInWorldFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumOfEstimatedWrenchInWorldFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;    
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Proper Rate of Change of Momentum - Sum of External Estimated Wrenches In World Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

save2pdf("rateOfMomentumVsEstimatedWrenchesInWorldFrame.pdf", fH,300);



