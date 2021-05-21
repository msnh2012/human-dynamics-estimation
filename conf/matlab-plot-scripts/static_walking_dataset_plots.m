close all;
clear all;
clc;

N=8;
C = linspecer(N);
L = lines(N);
lineStyles = linspecer(N,'qualitative');

%% Plot parameters
fontSize  = 30;
legendFontSize  = 30;
lineWidth = 6;


gravity = 9.81;
subjectMass = 79.5;
objectMass = 9.55;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'experiments_dataset';
dataset_source = 'FTShoes_Dataset';
subject = 'sub02';
dataset = 'walking';

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset,'/hde_with_sot/1e0'); 

%% Load data
withSOT = load(strcat(dataset_path,'/withSOT.mat'));

startIndex = 500;

%% Get the shortest sample dataset time
plot_time = withSOT.data.time(startIndex:end) - withSOT.data.time(startIndex);
plot_time = plot_time/1e9;

endIndex = startIndex + numel(plot_time) - 1;


withSOT_time = withSOT.data.time/1e9;

%% Indexes
totalDataSize = size(withSOT_time, 2);
startIndex = 500;
dataSize = ceil(totalDataSize * 0.8) - 1; 

%% Wrench Measurements Increase Under Load

withSOT_leftFeet_Measurements = withSOT.data.task2_wrenchMeasurementsInWorldFrame(1:6, startIndex: endIndex)';
withSOT_rightFeet_Measurements = withSOT.data.task2_wrenchMeasurementsInWorldFrame(7:12, startIndex: endIndex)';

withSOT_Feet_Measurements = withSOT_leftFeet_Measurements + withSOT_rightFeet_Measurements;

withSOT_leftHand_Measurements = withSOT.data.task1_wrenchMeasurementsInWorldFrame(13:18, startIndex: endIndex)';
withSOT_rightHand_Measurements = withSOT.data.task1_wrenchMeasurementsInWorldFrame(19:24, startIndex: endIndex)';

withSOT_Hands_Measurements = withSOT_leftHand_Measurements + withSOT_rightHand_Measurements;


%% Measured Object Mass
withoutSOT_measured_object_mass = [];
withSOT_measured_object_mass = [];

for i = 1:size(withSOT_Feet_Measurements, 1)
    withSOT_measured_object_mass(i) = norm(withSOT_Feet_Measurements(i,1:3));
end

withoutSOT_measured_object_mass = withoutSOT_measured_object_mass/gravity - subjectMass;
withSOT_measured_object_mass = withSOT_measured_object_mass/gravity - subjectMass;

%% Wrench Estimates

withSOT_leftHand_Estimates = withSOT.data.task2_wrenchEstimatesInWorldFrame(13:18, startIndex: endIndex)';
withSOT_rightHand_Estimates = withSOT.data.task2_wrenchEstimatesInWorldFrame(19:24, startIndex: endIndex)';

withSOT_Hand_Estimates = withSOT_leftHand_Estimates + withSOT_rightHand_Estimates;

%% Estimated Object Mass
withoutSOT_estimated_object_mass = [];
withSOT_estimated_object_mass = [];


for i = 1:size(withSOT_Hand_Estimates, 1)
    withSOT_estimated_object_mass(i) = norm(withSOT_Hand_Estimates(i,1:3));
end

withoutSOT_estimated_object_mass = withoutSOT_estimated_object_mass/gravity;
withSOT_estimated_object_mass = withSOT_estimated_object_mass/gravity;



fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,3);


colorSize = 10;

ax = [];

for s = 1:3
    
    ax(s) = nexttile;

    plot(plot_time, withSOT_leftHand_Measurements(:, s) , 'LineWidth', lineWidth, 'Color', C(1, :), 'LineStyle', '-');
    hold on;
    plot(plot_time, withSOT_leftHand_Estimates(:, s), 'Color', C(3,:), 'LineWidth', lineWidth);
    hold on;
    
    axis tight
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    if s == 1
        title("Left Hand", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    plot(plot_time, withSOT_rightHand_Measurements(:, s) , 'LineWidth', lineWidth,  'Color', C(1, :), 'LineStyle', '-');
    hold on;
    plot(plot_time, withSOT_rightHand_Estimates(:, s), 'Color', C(3,:), 'LineWidth', lineWidth);
    hold on;
    
    axis tight
    set (gca, 'FontSize' , fontSize)
  
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Right Hand", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    plot(plot_time, withSOT_leftHand_Measurements(:, s) + withSOT_rightHand_Measurements(:, s) , 'LineWidth', lineWidth,  'Color', C(1, :), 'LineStyle', '-');
    hold on;
    plot(plot_time, withSOT_leftHand_Estimates(:, s) + withSOT_rightHand_Estimates(:, s), 'Color', C(3, :), 'LineWidth', lineWidth);
    hold on;
    
    axis tight
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Hands", 'FontSize', fontSize, 'fontweight','bold');
    end
end

lh = legend('Measurement', 'Estimation with NCWE',...
       'FontSize', legendFontSize, 'Location', 'SouthOutside', 'NumColumns', 4);
lh.Layout.Tile = 'North';

%% Plot force norm
ax = nexttile([1 3]);

yline(objectMass, '-.', '9.55 Kg', 'LineWidth', lineWidth, 'FontSize', fontSize);
hold on;
plot(plot_time, withSOT_measured_object_mass, 'LineWidth', lineWidth, 'Color', C(4, :), 'LineStyle', '-');
hold on;
plot(plot_time, withSOT_estimated_object_mass, 'LineWidth', lineWidth, 'Color', C(3, :));
hold on;

xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
ylabel("Mass [$kg$]", 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
axis tight



lh1 = legend(ax, 'Ground Truth (9.55 Kg)', 'Using Increase in Feet FT Measurements', 'Estimation with NCWE',...
       'FontSize', legendFontSize, 'Location', 'NorthOutside', 'NumColumns', 4);
% lh1.Layout.Tile = 'North';


txt = title("Estimated Object Mass Value", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

txt = title(tl, "Hands Force Estimation with NCWE using Covariance of 1e0", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

%% Save figure
save2pdf(strcat(dataset_path, "/", strcat(dataset, "_object_mass_estimation.pdf")), fH, 600);