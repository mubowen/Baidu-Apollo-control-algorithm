function draw_traj_curvature(traj)
% 画出曲线上每点的曲率
% traj = [X, X, Theta, Radius, Curvature]

k = 10;   %显示曲率长度大小的比例

n = length(traj);   %traj鐨勯暱搴?

for i = 1 : 1 : n
    x = traj(i, 1);     %点的横坐标
    y = traj(i, 2);     %点的纵坐标?
    theta = traj(i, 3); %点的航向角
    r = traj(i, 4);     %点的半径
    c = traj(i, 5);     %点的曲率
    circle_theta = theta - pi / 2;  %该点在圆上对应的弧度
    xo = x - r * cos(circle_theta); %圆心的x坐标
    yo = y - r * sin(circle_theta); %圆心的y坐标
    
    xc = xo + (r + k * c) * cos(circle_theta);  %曲率点的x坐标
    yc = yo + (r + k * c) * sin(circle_theta);  %曲率点的y坐标
    
    plot([x, xc], [y, yc], 'b');     %链接该点与曲率点
    hold on;
end


