close all;
clear all;
clc;

%% Plot parameters
fontSize = 20;
lineWidth = 2;

parulacolors = colormap(parula);
pinkcolors   = colormap(pink);
wintercolors = colormap(winter);
bonecolors   = colormap(bone);
autumncolors = colormap(autumn);
summercolors = colormap(summer);

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
wrenchEstimatesLegendString = ["$\hat{f}_x [N]$", "$\hat{f}_y [N]$", "$\hat{f}_z [N]$","$\hat{m}_x [Nm]$", "$\hat{m}_y [Nm]$", "$\hat{m}_z [Nm]$"];
wrenchSourceName = ["Left Foot Wrench", "Right Foot Wrench", "Left Hand Wrench", "Right Hand Wrench"];
momentumLegendString = ["$^{B}\underline{\dot{h}}_{L_x}$", "$^{B}\underline{\dot{h}}_{L_y}$", "$^{B}\underline{\dot{h}}_{L_z}$",...
                        "$^{B}\underline{\dot{h}}_{\omega_x}$", "$^{B}\underline{\dot{h}}_{\omega_y}$", "$^{B}\underline{\dot{h}}_{\omega_z}$"];
                    
%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');


%% Measurement wrenches in base frame
LeftFootMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(1:6,:)';
RightFootMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(7:12,:)';
LeftHandMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(13:18,:)';
RightHandMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(19:24,:)';

sumWrenchMeasurementsInBaseFrame = LeftFootMeasuredWrenchInBaseFrame + RightFootMeasuredWrenchInBaseFrame +...
                                   LeftHandMeasuredWrenchInBaseFrame + RightHandMeasuredWrenchInBaseFrame;
                               
sumFeetWrenchMeasurementsInBaseFrame = LeftFootMeasuredWrenchInBaseFrame + RightFootMeasuredWrenchInBaseFrame;

RateOfChangeOfMomentumInBaseFrame = data.rateOfChangeOfMomentumInBaseFrame;

%% Feet wrench in base frame
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


%%   Rate of Change of Momentum In Base Frame Vs Sum of External Wrenches Measurements In Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6
    
    subplot(2,3,s);
    plot( RateOfChangeOfMomentumInBaseFrame(s,:)', 'LineWidth', lineWidth);
    hold on;
    plot(sumWrenchMeasurementsInBaseFrame(:,s), 'LineWidth', lineWidth, 'LineStyle', '--');
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