% 设置数据文件夹
data_folder = '/home/user/文档/';
files = dir(fullfile(data_folder, '1-weld_7.csv'));

% 初始化
all_z_data = {};
weld_labels = {};
medians = [];
means = [];

% 读取所有焊缝数据
for i = 1:length(files)
    % 读取CSV
    filepath = fullfile(data_folder, files(i).name);
    T = readtable(filepath);

    % 提取relative_z
    z = T.relative_z;

    % 保存数据
    all_z_data{end+1} = z;
    weld_labels{end+1} = sprintf('清根 %d', i);

    % 计算统计值
    medians(end+1) = median(z);
    means(end+1) = mean(z);
end

% --- 绘制箱线图 ---
figure('Position',[200,200,800,500]);
boxplot(cell2mat(all_z_data'), ...
    repelem(1:length(all_z_data), cellfun(@length,all_z_data)), ...
    'Labels', weld_labels, ...
    'Widths', 0.5, ...
    'Colors', [0.2 0.5 0.8]);

title('清根-1箱线图', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('相对 Z 值 (mm)', 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;

% --- 在箱线图上标注中位数和平均值 ---
hold on;
for i = 1:length(all_z_data)
    x_pos = i;
    text(x_pos, medians(i), sprintf('中位数=%.3f', medians(i)), ...
        'HorizontalAlignment','center', 'VerticalAlignment','bottom', ...
        'FontSize',10, 'Color',[0 0.5 0]);
    text(x_pos, means(i), sprintf('平均值=%.3f', means(i)), ...
        'HorizontalAlignment','center', 'VerticalAlignment','top', ...
        'FontSize',10, 'Color',[0.8 0 0]);
end
hold off;

% --- 保存图像 ---
saveas(gcf, fullfile(data_folder, '/home/user/文档/weld_1_boxplot.png'));
disp('✅ 箱线图已保存到 D:\weld_data\weld_z_boxplot.png');

% --- 打印统计值 ---
fprintf('清根统计结果:\n');
for i = 1:length(medians)
    fprintf('清根 %d: 中位数=%.4f, 平均值=%.4f\n', ...
        i, medians(i), means(i));
end