function draw_arrow(start_point, end_point)
% 画一箭头从start_point到end_point

% 输入:
% start_point : 起点[x, y]
% end_point : 终点[x, y]

K = 0.25;       %箭头比例系数
linewidth = 2;  %画线宽度

theta = pi / 8; %箭头角度
A1 = [cos(theta), -sin(theta);
      sin(theta), cos(theta)];  %旋转矩阵
theta = -theta;
A2 = [cos(theta), -sin(theta);
      sin(theta), cos(theta)];  %旋转矩阵

arrow = start_point' - end_point'; %向量

arrow_1 = A1 * arrow;
arrow_2 = A2 * arrow;
arrow_1 = K * arrow_1 + end_point';
arrow_2 = K * arrow_2 + end_point';

hold on
plot([start_point(1), end_point(1)], [start_point(2), end_point(2)],...
    'k', 'linewidth', linewidth);
plot([arrow_1(1), end_point(1)], [arrow_1(2), end_point(2)], 'k',...
    'linewidth', linewidth);
plot([arrow_2(1), end_point(1)], [arrow_2(2), end_point(2)], 'k',...
    'linewidth', linewidth);
grid minor
axis equal

