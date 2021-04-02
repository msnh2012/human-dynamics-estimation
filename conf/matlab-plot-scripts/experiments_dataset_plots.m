close all;
clear all;
clc;

N=8;
C = linspecer(N);
lineStyles = linspecer(N,'qualitative');

%% Plot parameters
fontSize  = 20;
legendFontSize  = 20;
lineWidth = 3;

gravity = 9.81;
subjectMass = 75.4;
objectMass = 9.55;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'experiments_dataset';
dataset_source = 'FTShoes_Dataset';
subject = 'sub02';
dataset = 'static/hde';

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset); 

%% Load data for the hands force/torque measurement covariance from 1e-4 to 1e4
dataset_dir_names = ["1e-4", "1e0", "1e4"];

experiments_dataset = [];

hands_wrench_measurements = [];
hands_wrench_estimates = [];

for i = 1:size(dataset_dir_names, 2)
    
    experiments_dataset(i).withoutSOT = load(strcat(dataset_path, '/',  dataset_dir_names(i),'/withoutSOT.mat'));
    experiments_dataset(i).withSOT = load(strcat(dataset_path, '/',  dataset_dir_names(i), '/withSOT.mat'));
    
    %% Get hands measurements vs estimates in world
    hands_wrench_measurements(i).withoutSOT.lefthand = experiments_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(13:18, : )'; 
    hands_wrench_measurements(i).withoutSOT.righthand = experiments_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(19:24, : )';
    hands_wrench_measurements(i).withoutSOT.sum = hands_wrench_measurements(i).withoutSOT.lefthand + hands_wrench_measurements(i).withoutSOT.righthand;
    
    for j = 1:size(hands_wrench_measurements(i).withoutSOT.sum, 1)
        hands_wrench_measurements(i).withoutSOT.force_norm(j) = norm(hands_wrench_measurements(i).withoutSOT.sum(j,1:3));
    end
    
    hands_wrench_measurements(i).withSOT.lefthand = experiments_dataset(i).withSOT.data.task2_wrenchMeasurementsInWorldFrame(13:18, : )'; 
    hands_wrench_measurements(i).withSOT.righthand = experiments_dataset(i).withSOT.data.task2_wrenchMeasurementsInWorldFrame(19:24, : )';
    hands_wrench_measurements(i).withSOT.sum = hands_wrench_measurements(i).withSOT.lefthand + hands_wrench_measurements(i).withSOT.righthand;
    
    for j = 1:size(hands_wrench_measurements(i).withSOT.sum, 1)
        hands_wrench_measurements(i).withSOT.force_norm(j) = norm(hands_wrench_measurements(i).withSOT.sum(j,1:3));
    end
    
    hands_wrench_estimates(i).withoutSOT.lefthand = experiments_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(13:18, : )'; 
    hands_wrench_estimates(i).withoutSOT.righthand = experiments_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(19:24, : )';
    hands_wrench_estimates(i).withoutSOT.sum = hands_wrench_estimates(i).withoutSOT.lefthand + hands_wrench_estimates(i).withoutSOT.righthand;
    
    for j = 1:size(hands_wrench_estimates(i).withoutSOT.sum, 1)
        hands_wrench_estimates(i).withoutSOT.force_norm(j) = norm(hands_wrench_estimates(i).withoutSOT.sum(j,1:3));
    end
    
    hands_wrench_estimates(i).withSOT.lefthand = experiments_dataset(i).withSOT.data.task2_wrenchEstimatesInWorldFrame(13:18, : )'; 
    hands_wrench_estimates(i).withSOT.righthand = experiments_dataset(i).withSOT.data.task2_wrenchEstimatesInWorldFrame(19:24, : )';
    hands_wrench_estimates(i).withSOT.sum = hands_wrench_estimates(i).withSOT.lefthand + hands_wrench_estimates(i).withSOT.righthand;
    
    for j = 1:size(hands_wrench_estimates(i).withSOT.sum, 1)
        hands_wrench_estimates(i).withSOT.force_norm(j) = norm(hands_wrench_estimates(i).withSOT.sum(j,1:3));
    end
   
end


%% Plot hands wrench estimation withoutSOT

fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,3);


colorSize = 10;
PlotsColorMap = [summer(colorSize); winter(colorSize); autumn(colorSize)];
LineStyles = ["-.", "-", "--", "-.x", "-o", ":", "-h"];

ax = [];


for s = 1:3
    
    ax(s) = nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withoutSOT.lefthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
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
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withoutSOT.righthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
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
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withoutSOT.sum(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
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

%% Plot force norm
nexttile([1 3])
for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withoutSOT.force_norm, LineStyles(i),...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

title("Hands Estimated Force Norm without NCWE", 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
ylabel('Force Norm $ [N]$', 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
axis tight


txt = title(tl, "Hands Wrench Estimation without NCWE", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Measurement Covariance')

%% Save figure
save2pdf(strcat(dataset_path + "hands_wrench_estimation_without_ncwe.pdf"), fH, 300);




% Plot hands wrench estimation withSOT
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,3);


colorSize = 10;
PlotsColorMap = [summer(colorSize); winter(colorSize); autumn(colorSize)];
LineStyles = ["-.", "-", "--", "-.x", "-o", ":", "-h"];

ax = [];


for s = 1:3
    
    ax(s) = nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withSOT.lefthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
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
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withSOT.righthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
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
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withSOT.sum(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
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

%% Plot force norm
nexttile([1 3])
for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withSOT.force_norm, LineStyles(i),...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

title("Hands Estimated Force Norm with NCWE", 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
ylabel('Force Norm $ [N]$', 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
axis tight


txt = title(tl, "Hands Wrench Estimation with NCWE", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Measurement Covariance')

%% Save figure
save2pdf(strcat(dataset_path + "hands_wrench_estimation_with_ncwe.pdf"), fH, 300);


%% Plot hands estimated force norm and object mass estimation
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(3,1);

ax1 = nexttile;
for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withoutSOT.force_norm, '-',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withSOT.force_norm, '-.',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

title("Hands Estimated Force Norm with and without NCWE", 'FontSize', fontSize, 'fontweight','bold');
ylabel('Force Norm $ [N]$', 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
axis tight
ylim([-5 inf])

lh = legend(ax1, dataset_dir_names, 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Measurement Covariance without NCWE')



ax2 = nexttile;
yline(objectMass, '--', '9.55 Kg', 'LineWidth', lineWidth, 'FontSize', fontSize);
hold on;

for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withoutSOT.force_norm/abs(gravity), '-',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withSOT.force_norm/abs(gravity), '-.',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

title("Estimated Object mass with and without NCWE", 'FontSize', fontSize, 'fontweight','bold');
xlabel('Samples', 'FontSize', fontSize);
ylabel('Object Mass $ [Kg]$', 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
axis tight
ylim([-20 inf])

lh = legend(ax2, ['','', '', '', dataset_dir_names], 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Measurement Covariance with NCWE')


nexttile
yline(objectMass, '--', '9.55 Kg', 'LineWidth', lineWidth, 'FontSize', fontSize);
hold on;

for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withSOT.force_norm/abs(gravity), '-.',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

title("Estimated Object mass with NCWE", 'FontSize', fontSize, 'fontweight','bold');
xlabel('Samples', 'FontSize', fontSize);
ylabel('Object Mass $ [Kg]$', 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
axis tight
ylim([-5 inf])

%% Save figure
save2pdf(strcat(dataset_path + "hands_wrench_estimation_object_mass_estimation.pdf"), fH, 300);