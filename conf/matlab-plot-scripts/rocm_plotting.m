close all;
clear all;
clc;

%% Plot parameters
fontSize = 20;
lineWidth = 2;

wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
momentumLegendString = ["$Lin_x [Kgm/s]$", "$Lin_y [Kgm/s]$", "$Lin_z [Kgm/s]$","$Ang_x [Nms]$", "$Ang_y [Nms]$", "$Ang_z [Nms]$"];

load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');
fH = figure('units','normalized','outerposition',[0 0 1 1]);
plot(data.rateOfChangeOfMomentumInBaseFrame(:,5:end)','LineWidth', lineWidth)
legend(momentumLegendString, 'Interpreter', 'latex')
set (gca, 'FontSize' , fontSize)
xlabel('Samples', 'FontSize', fontSize);

GravitationalWrenchInCentroidalFrame = ["${}^{\bar{G}}w$"];
GravitationalWrenchInBaseFrame = ["${}^{B}w = {}^{B}X^{*}_{\bar{G}} {}^{\bar{G}}w$"];
CentroidalMomentum = ["${}^{\bar{G}}h$"];
CentroidalMomentumBiasTerm = ["${}^{B}\dot{X}_{\bar{G}}^{*} \ {}^{\bar{G}}h$"];
ROCMInBaseAccelerationTerm = ["$ \sum \ {}^{B}\dot{X}_{L}^{*}  \ {}_{L}{I}_{L} \ {}^{L}{v}_{B,L}$"];
ROCMInBaseBiasTerm = ["$ \sum \ {}^{B}\dot{X}_{L}^{*}  \ {}_{L}{I}_{L} \ {}^{L}{v}_{B,L} $"];
TotalTerm = ["$ {}^{B}\dot{h}   - {}^{B}\dot{X}_{\bar{G}}^{*} \ {}^{\bar{G}}h - {}^{B}{X}_{\bar{G}}^{*} \ {}^{\bar{G}}{w} $"];


a = axes;
t = title(strcat(" ", ROCMInBaseAccelerationTerm),'Interpreter', 'latex');
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

%% Save figure
save2pdf("ROCMInBaseAccelerationTerm.pdf", fH,300);