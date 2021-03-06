close all;
clear all;
clc;

N=8;
C = linspecer(N);
lineStyles = linspecer(N,'qualitative');


%% Plot parameters
fontSize  = 30;
legendFontSize  = 20;
lineWidth = 3.5;

gravity = 9.81;
subjectMass = 79.5;
objectMass = 9.55;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset = 'static';

%% Load data
withoutSOT = load(strcat('/home/yeshi/software/robotology-superbuild/src/human-dynamics-estimation/conf/xml/testData/NCWE_MatData/',dataset ,'/withoutSOT.mat'));
withSOT = load(strcat('/home/yeshi/software/robotology-superbuild/src/human-dynamics-estimation/conf/xml/testData/NCWE_MatData/', dataset, '/withSOT.mat'));

withoutSOT_time = withoutSOT.data.time/1e9;
withSOT_time = withSOT.data.time/1e9;

%% Wrench Measurements Increase Under Load
withoutSOT_leftFeet_Measurements = withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(1:6,:)';
withoutSOT_rightFeet_Measurements = withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(7:12,:)';

withoutSOT_Feet_Measurements = withoutSOT_leftFeet_Measurements + withoutSOT_rightFeet_Measurements;

withSOT_leftFeet_Measurements = withSOT.data.task2_wrenchMeasurementsInWorldFrame(1:6,:)';
withSOT_rightFeet_Measurements = withSOT.data.task2_wrenchMeasurementsInWorldFrame(7:12,:)';

withSOT_Feet_Measurements = withSOT_leftFeet_Measurements + withSOT_rightFeet_Measurements;

withoutSOT_leftHand_Measurements = withoutSOT.data.task1_wrenchMeasurementsInWorldFrame(13:18,:)';
withoutSOT_rightHand_Measurements = withoutSOT.data.task1_wrenchMeasurementsInWorldFrame(19:24,:)';

withoutSOT_Hands_Measurements = withoutSOT_leftHand_Measurements + withoutSOT_rightHand_Measurements;

withSOT_leftHand_Measurements = withSOT.data.task1_wrenchMeasurementsInWorldFrame(13:18,:)';
withSOT_rightHand_Measurements = withSOT.data.task1_wrenchMeasurementsInWorldFrame(19:24,:)';

withSOT_Hands_Measurements = withSOT_leftHand_Measurements + withSOT_rightHand_Measurements;


%% Measured Object Mass
withoutSOT_measured_object_mass = [];
withSOT_measured_object_mass = [];

for i = 1:size(withoutSOT_Feet_Measurements, 1)
    withoutSOT_measured_object_mass(i) = norm(withoutSOT_Feet_Measurements(i,1:3));
end

for i = 1:size(withSOT_Feet_Measurements, 1)
    withSOT_measured_object_mass(i) = norm(withSOT_Feet_Measurements(i,1:3));
end

withoutSOT_measured_object_mass = withoutSOT_measured_object_mass/gravity - subjectMass;
withSOT_measured_object_mass = withSOT_measured_object_mass/gravity - subjectMass;

%% Wrench Estimates
withoutSOT_leftHand_Estimates = withoutSOT.data.task2_wrenchEstimatesInWorldFrame(13:18,:)';
withoutSOT_rightHand_Estimates = withoutSOT.data.task2_wrenchEstimatesInWorldFrame(19:24,:)';

withoutSOT_Hand_Estimates = withoutSOT_leftHand_Estimates + withoutSOT_rightHand_Estimates;

withSOT_leftHand_Estimates = withSOT.data.task2_wrenchEstimatesInWorldFrame(13:18,:)';
withSOT_rightHand_Estimates = withSOT.data.task2_wrenchEstimatesInWorldFrame(19:24,:)';

withSOT_Hand_Estimates = withSOT_leftHand_Estimates + withSOT_rightHand_Estimates;

%% Estimated Object Mass
withoutSOT_estimated_object_mass = [];
withSOT_estimated_object_mass = [];

for i = 1:size(withoutSOT_Hand_Estimates, 1)
    withoutSOT_estimated_object_mass(i) = norm(withoutSOT_Hand_Estimates(i,1:3));
end

for i = 1:size(withSOT_Hand_Estimates, 1)
    withSOT_estimated_object_mass(i) = norm(withSOT_Hand_Estimates(i,1:3));
end

withoutSOT_estimated_object_mass = withoutSOT_estimated_object_mass/gravity;
withSOT_estimated_object_mass = withSOT_estimated_object_mass/gravity;

%% Plot wrench forces
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(2,3);

nexttile;
plot(withSOT_leftHand_Measurements(:,1) + withSOT_rightHand_Measurements(:,1), 'LineWidth', lineWidth);
hold on;
plot(withoutSOT_leftHand_Estimates(:,1) + withoutSOT_rightHand_Estimates(:,1), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
plot(withSOT_leftHand_Estimates(:,1) + withSOT_rightHand_Estimates(:,1), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
%         ylim([-400 800])
xlabel('Samples', 'FontSize', fontSize);
ylabel(wrenchLegendString(1), 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
legend('Measured', 'Estimation without NCWE', 'Estimation with NCWE', 'FontSize', legendFontSize, 'Location', 'Best');
legend boxoff     

nexttile;
plot(withSOT_leftHand_Measurements(:,2) + withSOT_rightHand_Measurements(:,2), 'LineWidth', lineWidth);
hold on;
plot(withoutSOT_leftHand_Estimates(:,2) + withoutSOT_rightHand_Estimates(:,2), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
plot(withSOT_leftHand_Estimates(:,2) + withSOT_rightHand_Estimates(:,2), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
%         ylim([-400 800])
xlabel('Samples', 'FontSize', fontSize);
ylabel(wrenchLegendString(2), 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
legend('Measured', 'Estimation without NCWE', 'Estimation with NCWE', 'FontSize', legendFontSize, 'Location', 'Best');
legend boxoff     

nexttile;
plot(withSOT_leftHand_Measurements(:,3) + withSOT_rightHand_Measurements(:,3), 'LineWidth', lineWidth);
hold on;
plot(withoutSOT_leftHand_Estimates(:,3) + withoutSOT_rightHand_Estimates(:,3), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
plot(withSOT_leftHand_Estimates(:,3) + withSOT_rightHand_Estimates(:,3), 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
%         ylim([-400 800])
xlabel('Samples', 'FontSize', fontSize);
ylabel(wrenchLegendString(3), 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
legend('Measured', 'Estimation without NCWE', 'Estimation with NCWE', 'FontSize', legendFontSize, 'Location', 'Best');
legend boxoff     

txt = title(tl, "Sum of forces at the hands", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

%% Plots object mass value
nexttile([1 3])

yline(objectMass, '--', '9.55 Kg', 'LineWidth', lineWidth, 'FontSize', fontSize);
hold on;
plot(withSOT_measured_object_mass, 'LineWidth', lineWidth);
hold on;
plot(withoutSOT_estimated_object_mass, 'LineWidth', lineWidth);
hold on;
plot(withSOT_estimated_object_mass, 'LineWidth', lineWidth);
hold on;
% plot(abs(withSOT_measured_object_mass - withSOT_estimated_object_mass), 'LineWidth', lineWidth);

xlabel('Samples', 'FontSize', fontSize);
ylabel("Mass [$kg$]", 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
legend('Ground Truth (9.55 Kg)', 'Measurement', 'Estimation without NCWE',...
       'Estimation with NCWE', 'FontSize', legendFontSize, 'Location', 'Best');
legend boxoff     

txt = title("Object Mass Value", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

%% Save figure
save2pdf(strcat(dataset + ".pdf"), fH,300);