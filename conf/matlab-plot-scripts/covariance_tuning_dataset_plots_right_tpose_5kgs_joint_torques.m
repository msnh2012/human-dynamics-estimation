close all;
clear all;
clc;

N = 5;
C = linspecer(N);
L = lines(N);
lineStyles = linspecer(N,'qualitative');

tposeColor = [0.3333    0.4196    0.1843];

%% Plot parameters
fontSize  = 22;
legendFontSize  = 20;
lineWidth = 4;

gravity = 9.81;
objectMass = 5;

datasettype = 'with';

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';


dataset_type = 'covariance_tuning_dataset';
dataset_source = 'AMTI_Dataset';
subject = 'sub03';


dataset = strcat('tpose_right_5kgs/hde_', datasettype, '_sot');

subjectMass = 0;

if subject == 'sub02'

    subjectMass = 79.4;

elseif subject == 'sub03'
    
    subjectMass = 75.4;

end

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset,'/'); 

dataset_dir_names = ["1e-4", "1e-3", "1e-2", "1e-1", "1e0"];
% dataset_dir_names = ["1e0", "1e1", "1e2", "1e4"];


time_data = [];
plot_time = [];

for i = 1:size(dataset_dir_names, 2)
    
    covariance_tuning_dataset(i).Data = load(strcat(dataset_path, '/',  dataset_dir_names(i), '/ ', datasettype, 'SOT.mat'));
    
     %% Get time
    time_data(i).time = covariance_tuning_dataset(i).Data.data.time'/1e9;
    
end


startIndex = 400;

%% Get the shortest sample dataset time
plot_time = time_data(1).time(startIndex:end) - time_data(1).time(startIndex);

endIndex = startIndex + numel(plot_time) - 1;

jointPosition  = covariance_tuning_dataset(1).Data.data.jointPositions(42, startIndex : endIndex);
tposeRange = jointPosition < 0.5 & jointPosition > 0.25;
tposeStartIndex = find(tposeRange, 1, 'first');
tposeEndIndex   = find(tposeRange, 1, 'last');


%% Joint Torque Analysis
joint_names = covariance_tuning_dataset(1).Data.dynamicsJointNames;

joint_torque_estimates = [];

for i = 1:size(dataset_dir_names, 2)
    
    
    for j = 1:size(joint_names, 1)
        joint_torque_estimates(i).Data.(cell2mat(joint_names(j))).joint_torques = covariance_tuning_dataset(i).Data.data.jointTorques(j, startIndex : endIndex)';
    end
    
end


fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,2);


joint_name_list = ["jLeftShoulder", "jRightShoulder", "jLeftElbow", "jRightElbow", "jLeftWrist", "jRightWrist"];

tl = tiledlayout(4,size(joint_name_list, 2));

joint_suffix = ["_rotx", "_roty", "_rotz"];


ax = [];

for l = 1:size(joint_suffix, 2)
    
    for j = 1:size(joint_name_list,2)
        
        ax(j) = nexttile;
        
        for i = 1:size(dataset_dir_names, 2)
                        
            plot(plot_time, joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(l))).joint_torques, '-.',...
                'LineWidth', lineWidth,...
                'Color', C(i,:));
            hold on;
            
        end
        
        xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
            'LabelVerticalAlignment', 'bottom', 'Color', tposeColor);
        
        xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
            'LabelVerticalAlignment', 'bottom',  'Color', tposeColor);
        
        
        txt = title(strcat(joint_name_list(j), joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
        txt.Interpreter= 'none';
        
        
        if j == 1
            ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        end
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        
    end
    
end


for j = 1:size(joint_name_list,2)

    for i = 1:size(dataset_dir_names, 2)
        
        dataSize = size( joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques, 1);
        
        for e = 1:dataSize
            
            joint_torque_estimates(i).Data.joint_name_list(j).effort(e) = norm([joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques(e),...
                                                                                   joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(2))).joint_torques(e),...
                                                                                   joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(3))).joint_torques(e)]);
            
        end
        
        dataSize = size( joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques, 1);
        
        for e = 1:dataSize
            
            joint_torque_estimates(i).Data.joint_name_list(j).effort(e) = norm([joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques(e),...
                                                                                      joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(2))).joint_torques(e),...
                                                                                      joint_torque_estimates(i).Data.(strcat(joint_name_list(j), joint_suffix(3))).joint_torques(e)]);
            
        end
         
    end
    
end




joint_torques_benchmark = [0; 45.57; 0; 20.76; 0; 5.15];

axyline = [];

for j = 1:size(joint_name_list,2)
    
    nexttile
    
    if joint_torques_benchmark(j) ~= 0
        axyline = yline(joint_torques_benchmark(j), '-', num2str(joint_torques_benchmark(j)), 'LineWidth',...
          lineWidth, 'FontSize', fontSize, 'Color', 'k', 'LabelVerticalAlignment', 'bottom', 'FontSize', fontSize);
      hold on;
    end

        
    for i = 1:size(dataset_dir_names, 2)
        
        plot(plot_time, joint_torque_estimates(i).Data.joint_name_list(j).effort, '-.',...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
        
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
        if j == 1
            ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        end
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        title(strcat(joint_name_list(j), " Effort"), 'FontSize', fontSize, 'fontweight','bold');
        
    end
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom',  'Color', tposeColor);
    
end


lh = legend(ax(2), dataset_dir_names, 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Right Hand Measurement Covariance with NCWE', 'FontSize', fontSize)

lh = legend(axyline, 'Baseline Joint Effort', 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical');
lh.Layout.Tile = 'North';

txt = title(tl, "Joint Torque Estimation", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

%% Save figure
save2pdf(strcat(dataset_path + "joint_torque_estimates_tpose_5kgs_", datasettype, "_ncwe.pdf"), fH, 600);