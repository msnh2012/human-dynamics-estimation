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

%% Sum of offsetRemovedWrench measurements

sumOfOffsetRemovedMeasurements = data.wrenchEstimates(1:6,:)' + data.wrenchEstimates(13:18,:)' + data.wrenchEstimates(25:30,:)' + data.wrenchEstimates(37:42,:)';
sumOfEstimatedWrench = data.wrenchEstimates(7:12,:)' + data.wrenchEstimates(19:24,:)' + data.wrenchEstimates(31:36,:)' + data.wrenchEstimates(43:48,:)';

figure;
plot(sumOfOffsetRemovedMeasurements);
hold on;
title('sumOfOffsetRemovedMeasurements');
legend('$f_x [N]$', '$f_y [N]$', '$f_z [N]$','$m_x [Nm]$', '$m_y [Nm]$', '$m_z [Nm]$', 'Interpreter', 'latex', 'FontSize', 12);

figure;
plot(sumOfEstimatedWrench);
hold on;
title('sumOfEstimatedWrench');
legend('$f_x [N]$', '$f_y [N]$', '$f_z [N]$','$m_x [Nm]$', '$m_y [Nm]$', '$m_z [Nm]$', 'Interpreter', 'latex', 'FontSize', 12);

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
wrenchSourceName = {'Left Foot Wrench In Link Frame', 'Right Foot Wrench In Link Frame', 'Left Hand Wrench In Link Frame', 'Right Hand Wrench In Link Frame'};
momentumLegendString = ["$\dot{H}_{L_x}$", "$\dot{H}_{L_y}$", "$\dot{H}_{L_z}$", "$\dot{H}_{\omega_x}$", "$\dot{H}_{\omega_y}$", "$\dot{H}_{\omega_z}$"];


%% offSetRemovedMeasurement Vs Estimates

numberOfWrenchSources = 4;

for i = 1:numberOfWrenchSources
    
    fH = figure('units','normalized','outerposition',[0 0 1 1]);
    
    for s = 1:6
        
        subplot(2,3,s);
        plot(data.wrenchEstimates(s + 2 * 6 * (i-1),:)', 'LineWidth', lineWidth);
        hold on;
        plot(data.wrenchEstimates(6 + s + 2 * 6 * (i-1),:)', 'LineWidth', lineWidth, 'LineStyle', '--');
        hold on;
        xlabel('Samples', 'FontSize', fontSize);
        ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
        set (gca, 'FontSize' , fontSize)
        legend('Measured Wrench', 'Estimated Wrench', 'FontSize', fontSize, 'Location', 'Best');
        
    end
    
    a = axes;
    t = title (wrenchSourceName(i));
    t.FontSize = fontSize;
    a.Visible = 'off' ;
    t.Visible = 'on' ;
    
    %% Save figure
    save2pdf(strcat(wrenchSourceName(i) + ".pdf"), fH,300);
    
end


properRateOfChangeOfMomentumInWorldFrame = data.comProperAccelerationInWorldFrame;
properRateOfChangeOfMomentumInBaseFrame = data.comProperAccelerationInBaseFrame;

%% %% Proper Rate of Change of Momentum In Base Frame Vs Sum of External Wrenches In Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6 
    
    subplot(2,3,s);
    plot(properRateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumOfEstimatedWrench(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;    
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Proper Rate of Change of Momentum Vs Sum of External Wrenches In Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

save2pdf("rateOfMomentumVsWrenchesInBaseFrame.pdf", fH,300);


%% %% Proper Rate of Change of Momentum In World Frame Vs Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6 
    
    subplot(2,3,s);
    plot(properRateOfChangeOfMomentumInWorldFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(properRateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;    
    xlabel('Samples', 'FontSize', fontSize);
    legend(strcat("World - " + momentumLegendString(s)), strcat("Base - " + momentumLegendString(s)), 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Proper Rate of Change of Momentum In World Frame Vs Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

save2pdf("rateOfMomentumInWorldVsInBase.pdf", fH,300);