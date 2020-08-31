close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;

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

sumHandsWrenchMeasurementsInBaseFrame = LeftHandMeasuredWrenchInBaseFrame + RightHandMeasuredWrenchInBaseFrame;

RateOfChangeOfMomentumInBaseFrame = data.rateOfChangeOfMomentumInBaseFrame;

%% Feet wrench in base frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);
    
for s = 1:6
        
    subplot(2,3,s);
    plot(LeftFootMeasuredWrenchInBaseFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(RightFootMeasuredWrenchInBaseFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(sumFeetWrenchMeasurementsInBaseFrame(:,s), 'Color', summercolors(120,:), 'LineWidth', lineWidth);
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
    plot( RateOfChangeOfMomentumInBaseFrame(s,:)',  'Color', bonecolors(120,:), 'LineWidth', lineWidth);
    hold on;
    plot(sumWrenchMeasurementsInBaseFrame(:,s), 'Color', summercolors(120,:), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;
%     ylim([-400 1000])
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s),...
           'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Rate of Change of Momentum - Sum of External Wrenches Measurements In Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;



%% Estimate wrenches in base frame
LeftFootEstimatedWrenchInBaseFrame  = data.task1_wrenchEstimatesInBaseFrame(1:6,:)';
RightFootEstimatedWrenchInBaseFrame = data.task1_wrenchEstimatesInBaseFrame(7:12,:)';
LeftHandEstimatedWrenchInBaseFrame  = data.task1_wrenchEstimatesInBaseFrame(13:18,:)';
RightHandEstimatedWrenchInBaseFrame = data.task1_wrenchEstimatesInBaseFrame(19:24,:)';

sumWrenchEstimatesInBaseFrame = LeftFootEstimatedWrenchInBaseFrame + RightFootEstimatedWrenchInBaseFrame +...
                                LeftHandEstimatedWrenchInBaseFrame + RightHandEstimatedWrenchInBaseFrame;
                            
sumFeetWrenchEstimatesInBaseFrame = LeftFootEstimatedWrenchInBaseFrame + RightFootEstimatedWrenchInBaseFrame;
                            
sumHandsWrenchEstimatesInBaseFrame = LeftHandEstimatedWrenchInBaseFrame + RightHandEstimatedWrenchInBaseFrame;


%%   Rate of Change of Momentum In Base Frame Vs Sum of External Wrenches Measurements In Base Frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6
    
    subplot(2,3,s);
    plot( RateOfChangeOfMomentumInBaseFrame(s,:)',  'Color', bonecolors(120,:), 'LineWidth', lineWidth);
    hold on;
    plot(sumWrenchMeasurementsInBaseFrame(:,s), 'Color', summercolors(120,:), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;
    plot(sumWrenchEstimatesInBaseFrame(:,s), 'Color', pinkcolors(40,:), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;
%     ylim([-400 1000])
    xlabel('Samples', 'FontSize', fontSize);
    legend(momentumLegendString(s), wrenchLegendString(s), wrenchEstimatesLegendString(s),...
           'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
end

a = axes;
t = title ("Rate of Change of Momentum - Sum of External Wrenches Measurements and Estimates In Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;


sumOfWrenchMeasurementsInBaseNorm = [];
sumOfWrenchEstimatesInBaseNorm    = [];

for i=1:size(sumWrenchMeasurementsInBaseFrame, 1)
    sumOfWrenchMeasurementsInBaseNorm(i) = norm(sumWrenchMeasurementsInBaseFrame(i,1:3));
end

for i=1:size(sumWrenchEstimatesInBaseFrame, 1)
    sumOfWrenchEstimatesInBaseNorm(i) = norm(sumWrenchEstimatesInBaseFrame(i,1:3));
end

%%  Sum of force measurements and estimates norm
fH = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(2,1,1)
plot( sumOfWrenchMeasurementsInBaseNorm,  'Color', autumncolors(80,:), 'LineWidth', lineWidth);
hold on;
plot(sumOfWrenchEstimatesInBaseNorm, 'Color', wintercolors(160,:), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Norm of Sum of force measurements', 'Norm of Sum of force estimates', ...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');

subplot(2,1,2)
plot(sumOfWrenchMeasurementsInBaseNorm - sumOfWrenchEstimatesInBaseNorm, 'Color', bonecolors(120,:), 'LineWidth', lineWidth);
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Difference',...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
a = axes;
t = title ("Norm of Sum of force measurements and estimates expressed in base frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;


sumOfFeetWrenchMeasurementsInBaseNorm = [];
sumOfFeetWrenchEstimatesInBaseNorm    = [];

for i=1:size(sumFeetWrenchMeasurementsInBaseFrame, 1)
    sumOfFeetWrenchMeasurementsInBaseNorm(i) = norm(sumFeetWrenchMeasurementsInBaseFrame(i,1:3));
end

for i=1:size(sumFeetWrenchEstimatesInBaseFrame, 1)
    sumOfFeetWrenchEstimatesInBaseNorm(i) = norm(sumFeetWrenchEstimatesInBaseFrame(i,1:3));
end

%%  Sum of feet force measurements and estimates norm
fH = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(2,1,1)
plot( sumOfFeetWrenchMeasurementsInBaseNorm,  'Color', autumncolors(80,:), 'LineWidth', lineWidth);
hold on;
plot(sumOfFeetWrenchEstimatesInBaseNorm, 'Color', wintercolors(160,:), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Norm of Sum of feet force measurements', 'Norm of Sum of feet force estimates', ...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');

subplot(2,1,2)
plot(sumOfFeetWrenchMeasurementsInBaseNorm - sumOfFeetWrenchEstimatesInBaseNorm, 'Color', bonecolors(120,:), 'LineWidth', lineWidth);
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Difference',...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
a = axes;
t = title ("Norm of Sum of feet force measurements and estimates expressed in base frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;


sumOfHandsWrenchMeasurementsInBaseNorm = [];
sumOfHandsWrenchEstimatesInBaseNorm    = [];

for i=1:size(sumHandsWrenchMeasurementsInBaseFrame, 1)
    sumOfHandsWrenchMeasurementsInBaseNorm(i) = norm(sumHandsWrenchMeasurementsInBaseFrame(i,1:3));
end

for i=1:size(sumHandsWrenchEstimatesInBaseFrame, 1)
    sumOfHandsWrenchEstimatesInBaseNorm(i) = norm(sumHandsWrenchEstimatesInBaseFrame(i,1:3));
end

%%  Sum of hands force measurements and estimates norm
fH = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(2,1,1)
plot( sumOfHandsWrenchMeasurementsInBaseNorm,  'Color', autumncolors(80,:), 'LineWidth', lineWidth);
hold on;
plot(sumOfHandsWrenchEstimatesInBaseNorm, 'Color', wintercolors(160,:), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Norm of Sum of hands force measurements', 'Norm of Sum of hands force estimates', ...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');

subplot(2,1,2)
plot(sumOfHandsWrenchMeasurementsInBaseNorm - sumOfHandsWrenchEstimatesInBaseNorm, 'Color', bonecolors(120,:), 'LineWidth', lineWidth);
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Difference',...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
    
a = axes;
t = title ("Norm of Sum of hands force measurements and estimates expressed in base frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;


LeftHandEstimatedWrenchInBaseFrameNorm  = [];
RightHandEstimatedWrenchInBaseFrameNorm = [];

for i=1:size(LeftHandEstimatedWrenchInBaseFrame, 1)
    LeftHandEstimatedWrenchInBaseFrameNorm(i) = norm(LeftHandEstimatedWrenchInBaseFrame(i,1:3));
end

for i=1:size(RightHandEstimatedWrenchInBaseFrame, 1)
    RightHandEstimatedWrenchInBaseFrameNorm(i) = norm(RightHandEstimatedWrenchInBaseFrame(i,1:3));
end

%% Estimate wrenches in link frame
LeftFootEstimatedWrenchInLinkFrame  = data.task1_wrenchEstimatesInLinkFrame(1:6,:)';
RightFootEstimatedWrenchInLinkFrame = data.task1_wrenchEstimatesInLinkFrame(7:12,:)';
LeftHandEstimatedWrenchInLinkFrame  = data.task1_wrenchEstimatesInLinkFrame(13:18,:)';
RightHandEstimatedWrenchInLinkFrame = data.task1_wrenchEstimatesInLinkFrame(19:24,:)';

LeftHandEstimatedWrenchInLinkFrameNorm  = [];
RightHandEstimatedWrenchInLinkFrameNorm = [];

for i=1:size(LeftHandEstimatedWrenchInLinkFrame, 1)
    LeftHandEstimatedWrenchInLinkFrameNorm(i) = norm(LeftHandEstimatedWrenchInLinkFrame(i,1:3));
end

for i=1:size(RightHandEstimatedWrenchInBaseFrame, 1)
    RightHandEstimatedWrenchInLinkFrameNorm(i) = norm(RightHandEstimatedWrenchInLinkFrame(i,1:3));
end


%%  hands force estimates in base frame norm
fH = figure('units','normalized','outerposition',[0 0 1 1]);


subplot(2,1,1)
plot( LeftHandEstimatedWrenchInBaseFrameNorm,  'Color', autumncolors(80,:), 'LineWidth', lineWidth);
hold on;
plot( LeftHandEstimatedWrenchInLinkFrameNorm,  'Color', autumncolors(180,:), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
plot(RightHandEstimatedWrenchInBaseFrameNorm, 'Color', wintercolors(160,:), 'LineWidth', lineWidth);
hold on;
plot(RightHandEstimatedWrenchInLinkFrameNorm, 'Color', wintercolors(200,:), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Norm of Left Hand force estimates in base', 'Norm of Left Hand force estimates in link', ... 
       'Norm of Right Hand force estimates in base', 'Norm of Right Hand force estimates in link', ...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');

subplot(2,1,2)
plot(LeftHandEstimatedWrenchInBaseFrameNorm + RightHandEstimatedWrenchInBaseFrameNorm, 'Color', bonecolors(120,:), 'LineWidth', lineWidth);
hold on;
plot(LeftHandEstimatedWrenchInLinkFrameNorm + RightHandEstimatedWrenchInLinkFrameNorm, 'Color', summercolors(10,:), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
xlabel('Samples', 'FontSize', fontSize);
legend('Base Frame Sum', 'Link Frame Sum', ...
       'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
   
     
a = axes;
t = title ("Norm of Sum of hands force estimates expressed in base and link frames");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;



