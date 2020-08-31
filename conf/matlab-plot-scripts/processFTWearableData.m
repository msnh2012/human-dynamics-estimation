close all;
clear all;
clc;

%% Load FT Wearable data and extract FT values
FTLeftShoeTable = readtable('/home/yeshi/Desktop/prash_hde_estimation_dataset/stati-Npose/trial3/wearable/FTshoes/left/data.log');
FTRightShoeTable = readtable('/home/yeshi/Desktop/prash_hde_estimation_dataset/stati-Npose/trial3/wearable/FTshoes/right/data.log');

FTLeftShoe = table;
FTRightShoe = table;

%% Check size differences
size_diff = size(FTLeftShoeTable, 1) - size(FTRightShoeTable , 1);
if (size_diff < 0)
    FTLeftShoe  = FTLeftShoeTable;
    FTRightShoe = FTRightShoeTable(1:size(FTLeftShoeTable, 1), :);
elseif (size_diff > 0)
    FTLeftShoe = FTLeftShoeTable(1:size(FTRightShoeTable, 1), :);
    FTRightShoe  = FTRightShoeTable;
end

FTLeftShoeData = {};
FTLeftShoeData.t = FTLeftShoe.Var2;
FTLeftShoeData.time = FTLeftShoe.Var3;

FTLeftShoeData.FT = [];

FTLeftShoeData.FT(:,1) = FTLeftShoe.Var11;
FTLeftShoeData.FT(:,2) = FTLeftShoe.Var12;
FTLeftShoeData.FT(:,3) = FTLeftShoe.Var13;
FTLeftShoeData.FT(:,4) = FTLeftShoe.Var14;
FTLeftShoeData.FT(:,5) = FTLeftShoe.Var15;
FTLeftShoeData.mz      = FTLeftShoe.Var16;

for i = 1:3
    FTLeftShoeData.mz = regexprep(FTLeftShoeData.mz, '.$', '', 'lineanchors');
end

FTLeftShoeData.FT(:,6) = str2double(FTLeftShoeData.mz);


FTRightShoeData = {};
FTRightShoeData.t = FTRightShoe.Var2;
FTRightShoeData.time = FTRightShoe.Var3;

FTRightShoeData.FT = [];

FTRightShoeData.FT(:,1) = FTRightShoe.Var11;
FTRightShoeData.FT(:,2) = FTRightShoe.Var12;
FTRightShoeData.FT(:,3) = FTRightShoe.Var13;
FTRightShoeData.FT(:,4) = FTRightShoe.Var14;
FTRightShoeData.FT(:,5) = FTRightShoe.Var15;
FTRightShoeData.mz      = FTRightShoe.Var16;

for i = 1:3
    FTRightShoeData.mz = regexprep(FTRightShoeData.mz, '.$', '', 'lineanchors');
end

FTRightShoeData.FT(:,6) = str2double(FTRightShoeData.mz);


%% Plotting

fontSize = 20;
lineWidth = 2;
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];


base_X_leftfoot_ = [ 0.923054 0.228576 0.309392 0 0 0 
                      0.245455 0.969272 -0.0162123 0 0 0
                      0-0.29618 0.0909067 0.950796 0 0 0
                      0.15096 0.812686 0.150023 0.923054 0.228576 0.309392
                      -0.822251 0.207361 -0.051571 0.245455 0.969272 -0.0162123 
                      -0.210957 -0.167524 -0.0496973 -0.29618 0.0909067 0.950796]
                  
FTLeftShoeData.FT_in_base = base_X_leftfoot_ * FTLeftShoeData.FT'


fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6
    
    subplot(2,3,s);
    plot(FTLeftShoeData.FT(:,s), 'LineWidth', lineWidth);
    hold on;
%     plot(FTLeftShoeData.FT_in_base(s,:), 'LineWidth', lineWidth);
%     hold on;
    plot(FTRightShoeData.FT(:,s), 'LineWidth', lineWidth);
    hold on;
%     ylim([-400 800])
    
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Left Foot', 'Right Foot', 'FontSize', fontSize, 'Location', 'Best');

end
    

a = axes;
t = title ("Left Foot and Right Foot FT Values in Link Frame from Wearable Data");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;


%% Compute Norm
subjectWeight = 75.5 * 9.81;
FTLeftShoesData.ForceNorm = [];

for i = 1:size(FTLeftShoeData.FT, 1)
    FTLeftShoesData.ForceNorm(i) = norm(FTLeftShoeData.FT(i, 1:3));
end

FTRightShoesData.ForceNorm = [];

for i = 1:size(FTRightShoeData.FT, 1)
    FTRightShoesData.ForceNorm(i) = norm(FTRightShoeData.FT(i, 1:3));
end

fH = figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,1,1)
plot(FTLeftShoesData.ForceNorm', 'LineWidth', lineWidth);
hold on;
plot(FTRightShoesData.ForceNorm', 'LineWidth', lineWidth);
hold on;
yline(subjectWeight/2, 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
ylim([100 900])
xlabel('Samples', 'FontSize', fontSize);
ylabel('$Weight$', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('Left Foot Measured Weight $|| {}^{\mathcal{LF}}{f}_{LF} || $',...
       'Right Foot Measured Weight $|| {}^{\mathcal{RF}}{f}_{RF} ||$',...
       '$\frac{Subject Weight}{2}$', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');
   
subplot(2,1,2)
plot(FTLeftShoesData.ForceNorm' + FTRightShoesData.ForceNorm', 'LineWidth', lineWidth);
hold on;
yline(subjectWeight, 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
ylim([600 900])
xlabel('Samples', 'FontSize', fontSize);
ylabel('$Weight$', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('Feet Measured Weight $|| {}^{\mathcal{LF}}{f}_{LF} || + || {}^{\mathcal{RF}}{f}_{RF} ||$',...
       'Subject Weight', 'Interpreter', 'latex', 'FontSize', fontSize, 'Location', 'Best');

a = axes;
t = title ("Norm of FT Values in Link Frame from Wearable Data");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;


