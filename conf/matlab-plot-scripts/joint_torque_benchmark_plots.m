close all;
clear all;
clc;

N=8;
C = linspecer(N);
lineStyles = linspecer(N,'qualitative');

tposeColor = [0.3333    0.4196    0.1843];

%% Plot parameters
fontSize  = 25;
legendFontSize  = 30;
lineWidth = 4;

gravity = 9.81;
objectMass = 9.55;


%% Load urdf model
subjectModel = 'humanSubject03';
modelPath = strcat('/home/yeshi/software/robotology-superbuild/src/human-gazebo/', subjectModel, '/');
modelFileName  = strcat(subjectModel, '_66dof.urdf');


%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'covariance_tuning_dataset';
dataset_source = 'AMTI_Dataset';
subject = 'sub03';

% % %% Tpose dataset
% % dataset = 'tpose/hde/feet';
% % dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset, '/');
% % Data =  load(strcat(dataset_path,'1e-6/withoutSOT.mat'));
% % startIndex = 100;
% % joint_torques_benchmark = [12.94; 3.46; 0.43];

%% Tpose 5 Kgs
dataset = 'tpose_right_5kgs/hde_with_sot';
dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset, '/');
Data =  load(strcat(dataset_path,'1e-2/withSOT.mat'));
startIndex = 500;
joint_torques_benchmark = [45.57; 20.76; 5.15];


joint_names = Data.dynamicsJointNames(1:66);


%% Get computed torques from iDynTree Inverse Dynamics
[inverseDynamicsJointTorques, gravityTrqs, biasTrqs, baseWrench] = computeIDJointTorques(Data, modelPath, modelFileName, joint_names);



plot_time = Data.data.time(startIndex:end)'/1e9;

joint_torque_estimates = [];

jointPosition  = Data.data.jointPositions(42, startIndex : end);
% % tposeRange = jointPosition < 0.15 & jointPosition > 0.12;

tposeRange = jointPosition < 0.5 & jointPosition > 0.25;

tposeStartIndex = find(tposeRange, 1, 'first');
tposeEndIndex   = find(tposeRange, 1, 'last');



for j = 1:size(joint_names, 1)
    joint_torque_estimates.(cell2mat(joint_names(j))).joint_torques = Data.data.jointTorques(j, startIndex:end)';
    joint_torque_estimates.(cell2mat(joint_names(j))).ID_joint_torques = inverseDynamicsJointTorques(j,  startIndex:end)';
    joint_torque_estimates.(cell2mat(joint_names(j))).gravity_joint_torques = gravityTrqs(j,  startIndex:end)';
end



%% Plot
fH = figure('units','normalized','outerposition',[0 0 1 1]);

joint_names_list = ["Shoulder", "Elbow", "Wrist"];
joint_suffix = ["_rotx", "_roty", "_rotz"];

tl = tiledlayout(4, 2 * size(joint_names_list, 2));

cmap = colormap(parula);
ID_joint_toruqe_color = cmap(200, :);

for l = 1:size(joint_suffix, 2)
    
    for j = 1:size(joint_names_list, 2)
        
        
        nexttile;
        
        plot(plot_time, joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(l))).joint_torques, '-',...
            'LineWidth', lineWidth,...
            'Color', C(l,:));
        hold on;
        
        plot(plot_time, joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(l))).ID_joint_torques, '--',...
            'LineWidth', lineWidth,...
            'Color', ID_joint_toruqe_color);
        hold on;

        xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
            'LabelVerticalAlignment', 'middle', 'Color', tposeColor);
        
        xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
            'LabelVerticalAlignment', 'middle',  'Color', tposeColor);
        
        txt = title(strcat("jLeft",joint_names_list(j), joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
        txt.Interpreter= 'none';
        
        if j == 1 && (joint_suffix(l) == "_rotx")
            ylabel("$\tau_x$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        elseif j == 1 && (joint_suffix(l) == "_roty")
            ylabel("$\tau_y$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        elseif j == 1 && (joint_suffix(l) == "_rotz")
            ylabel("$\tau_z$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize); 
        end
        
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        nexttile;
        
        plot(plot_time, joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(l))).joint_torques, '-',...
            'LineWidth', lineWidth,...
            'Color', C(l,:));
        hold on;
        
        plot(plot_time, joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(l))).ID_joint_torques, '--',...
            'LineWidth', lineWidth,...
            'Color', ID_joint_toruqe_color);
        hold on;
        
        xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
            'LabelVerticalAlignment', 'bottom', 'Color', tposeColor);
        
        xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
            'LabelVerticalAlignment', 'bottom',  'Color', tposeColor);
        
        txt = title(strcat("jRight",joint_names_list(j), joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
        txt.Interpreter= 'none';
        
        
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        
    end
    
    if (joint_suffix(l) == "_rotx")
        lh = legend("$\tau_x$", 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical', 'Interpreter', 'latex', 'Box','off');
        lh.Layout.Tile = 'North';
    elseif (joint_suffix(l) == "_roty")
        lh = legend("$\tau_y$", 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical', 'Interpreter', 'latex', 'Box','off');
        lh.Layout.Tile = 'North';
    elseif (joint_suffix(l) == "_rotz")
        lh = legend("$\tau_z$", 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical', 'Interpreter', 'latex', 'Box','off');
        lh.Layout.Tile = 'North';
    end
    
    
end




for j = 1:size(joint_names_list, 2)
    
    dataSize = size( joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(1))).joint_torques, 1);
    
    for e = 1:dataSize
        
        joint_torque_estimates.(strcat("jLeft", joint_names_list(j))).effort(e) = norm([joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(1))).joint_torques(e),...
            joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(2))).joint_torques(e),...
            joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(3))).joint_torques(e)]);
        
        joint_torque_estimates.(strcat("jRight", joint_names_list(j))).effort(e) = norm([joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(1))).joint_torques(e),...
            joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(2))).joint_torques(e),...
            joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(3))).joint_torques(e)]);
        
        joint_torque_estimates.(strcat("jLeft", joint_names_list(j))).ID_effort(e) = norm([joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(1))).ID_joint_torques(e),...
            joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(2))).ID_joint_torques(e),...
            joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(3))).ID_joint_torques(e)]);
        
        joint_torque_estimates.(strcat("jRight", joint_names_list(j))).ID_effort(e) = norm([joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(1))).ID_joint_torques(e),...
            joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(2))).ID_joint_torques(e),...
            joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(3))).ID_joint_torques(e)]);
        
    end
    
    nexttile
    plot(plot_time, joint_torque_estimates.(strcat("jLeft", joint_names_list(j))).effort, '-',...
        'LineWidth', lineWidth,...
        'Color', C(6,:));
    hold on;
        
    ax1 = plot(plot_time, joint_torque_estimates.(strcat("jLeft", joint_names_list(j))).ID_effort, '-',...
        'LineWidth', lineWidth,...
        'Color', ID_joint_toruqe_color);
    hold on;
    
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom',  'Color', tposeColor);
    
% %     yline(joint_torques_benchmark(j), '-', num2str(joint_torques_benchmark(j)), 'LineWidth',...
% %         lineWidth, 'FontSize', fontSize, 'Color', 'k', 'LabelVerticalAlignment', 'bottom', 'FontSize', fontSize);
% %     hold on;
    
    title(strcat("jLeft",joint_names_list(j), " Effort"), 'FontSize', fontSize, 'fontweight','bold');
    
    xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    
    if j == 1 
        ylabel("$\tau_{effort}$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        lh = legend('$\tau_{effort}$', '', '', '', '', 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical', 'Interpreter', 'latex', 'Box','off');
        lh.Layout.Tile = 'North';
    end
    
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    ylim([0 inf]);
    
    nexttile
    ax = yline(joint_torques_benchmark(j), '-', num2str(joint_torques_benchmark(j)), 'LineWidth',...
        lineWidth, 'FontSize', fontSize, 'Color', 'k', 'LabelVerticalAlignment', 'bottom', 'FontSize', fontSize);
    hold on;
    plot(plot_time, joint_torque_estimates.(strcat("jRight", joint_names_list(j))).effort, '-',...
        'LineWidth', lineWidth,...
        'Color', C(6,:));
    hold on;
    
    plot(plot_time, joint_torque_estimates.(strcat("jRight", joint_names_list(j))).ID_effort, '-',...
        'LineWidth', lineWidth,...
        'Color', ID_joint_toruqe_color);
    hold on;
    
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom',  'Color', tposeColor);
    
    title(strcat("jRight",joint_names_list(j), " Effort"), 'FontSize', fontSize, 'fontweight','bold');
    
    xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    
    hold on;
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    ylim([0 inf]);
end


lh = legend(ax1, 'Computed Torques', 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical', 'Box','off');
lh.Layout.Tile = 'North';


lh = legend(ax, 'Baseline Joint Effort', 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical', 'Box','off');
lh.Layout.Tile = 'North';

% % txt = title(tl, strcat("Joint Torque Estimation"), 'FontSize', fontSize, 'fontweight','bold');
% % txt.Interpreter= 'none';

txt = title(tl, strcat("Joint Torque Estimation with NCWE using Covariance of 1e0"), 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none';

%% Save figure
save2pdf(strcat(dataset_path + "joint_torque_estimates.pdf"), fH, 600);

% % %% Plot base wrench
% % wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
% % 
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % 
% % tl = tiledlayout(1,1);
% % 
% % ax = plot(plot_time, baseWrench(:,startIndex:end)', 'LineWidth', lineWidth);
% % hold on;
% % 
% % xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
% % 
% % 
% % legend(ax, wrenchLegendString, 'FontSize', legendFontSize,...
% %     'Location', 'NorthOutside', 'Orientation','Vertical', 'NumColumns', 6, 'Interpreter', 'Latex');
% % 
% % txt = title(tl, "iDynTree Inverse Dynamics Base Wrench", 'FontSize', fontSize, 'fontweight','bold');
