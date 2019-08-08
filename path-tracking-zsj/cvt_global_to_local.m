function point_local = cvt_global_to_local(point_global, base_local)
% 坐标系转换：将点的坐标从全局坐标系转为局部坐标系

% 输出:
% point_local: 局部坐标系的点坐标[x_local, y_local]

% 输入:
% point_global: 全局坐标系的点坐标[x_global, y_global]
% base_local  : 局部坐标系在全局坐标系的位置和角度[x_base, y_base, theta_base]

base_point = base_local(1:2);   %局部坐标系的原点坐标
base_theta = base_local(3);     %局部坐标系X轴与全局坐标系X轴的相对角度

% 平移变换
tmp_point = point_global - base_point;

% 旋转变换
A = [cos(base_theta), -sin(base_theta); 
     sin(base_theta),  cos(base_theta)];
point_local = tmp_point * A;

