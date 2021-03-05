close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;

gravity = 9.81;
subjectMass = 79.5;
objectMass = 9.55;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

%% Load data
withoutSOT = load('/home/yeshi/software/robotology-superbuild/src/human-dynamics-estimation/conf/xml/testData/NCWE_MatData/static/withoutSOT.mat');
withSOT = load('/home/yeshi/software/robotology-superbuild/src/human-dynamics-estimation/conf/xml/testData/NCWE_MatData/static/withSOT.mat');

withoutSOT_time = withoutSOT.data.time/1e9;
withSOT_time = withSOT.data.time/1e9;

%% Wrench Measurements Increase Under Load
withoutSOT_leftFeet_Measurements = withoutSOT.data.task2_wrenchEstimatesInWorldFrame(1:6,:)';
withoutSOT_rightFeet_Measurements = withoutSOT.data.task2_wrenchEstimatesInWorldFrame(7:12,:)';

withoutSOT_Feet_Measurements = withoutSOT_leftFeet_Measurements + withoutSOT_rightFeet_Measurements;

withSOT_leftFeet_Measurements = withSOT.data.task2_wrenchEstimatesInWorldFrame(1:6,:)';
withSOT_rightFeet_Measurements = withSOT.data.task2_wrenchEstimatesInWorldFrame(7:12,:)';

withSOT_Feet_Measurements = withSOT_leftFeet_Measurements + withSOT_rightFeet_Measurements;

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


%% Plots
fH = figure('units','normalized','outerposition',[0 0 1 1]);


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
legend('Ground Truth (9.55 Kg)', 'Measurement', 'Estimation without NCWE',...
       'Estimation with NCWE', 'FontSize', fontSize, 'Location', 'Best');

a = axes;
t = title ("Object Mass Value");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;