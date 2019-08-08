function veh_pose = update_veh_pose_mpc(origin_pose, steer_angle,...
    steer_acc,veh_params,mpc_params, time_step)

% 更新车辆的位姿状态 vehicle_pose = [x, y, theta]
% x     : 横坐标, m
% y     : 纵坐标, m
% theta : 航向角，与横坐标的角度，逆时针[0, 2*pi], rad

% 输入:
% origin_pose   : 初始车辆位姿状态
% steer_angle   : 前轮偏角, rad
% veh_params    : 车辆参数
% time_step     : 仿真步长, s

x0 = origin_pose(1);
y0 = origin_pose(2);
theta0 = origin_pose(3);

veh_params.velocity=veh_params.velocity + steer_acc* time_step ;
v=veh_params.velocity;

%% 用动力学更新汽车位姿？？？
% Ac=mpc_params.A;
% % Update the velocity of the Matrix A
% Ac(2,2)=Ac(2,2)/v;
% Ac(2,4)=Ac(2,3)/v;
% Ac(4,2)=Ac(4,2)/v;
% Ac(4,4)=Ac(4,4)/v;
%  
% Bc=mpc_params.B;
% Cc=mpc_params.C;
% % update matrix C
% Cc(2,1)=(mpc_params.lr*mpc_params.cr-...
%      mpc_params.lf*mpc_params.cf)/(mpc_params.mass*v)-v;
% Cc(4,1)=-(mpc_params.lf*mpc_params.lf*...
%      mpc_params.cf+mpc_params.lr*mpc_params.lr*...
%      mpc_params.cr)/(mpc_params.iz*v);
% 
% % Update matrix D
% n = size(Ac,1); % number of states
% p = size(Bc,2); % number of input
% q = size(Cc,1); % number of output
% Dc = zeros(q,p);
% 
%  [Ad,Bd,Cd,Dd] = c2dm(Ac,Bc,Cc,Dc,time_step);

%% 

 delta_dist = v * time_step;
 
% [y_k,y_k1,theta_k,theta_k1]=Ad*[delta_dist;v;theta0;veh_params.angular_v]...
%     +Bd.*steer_angle;
%%

  %车辆在单位仿真步长走过的距离

tol = 0.0001;   %判断直行还是转弯的前轮偏角阈值

if abs(steer_angle) > tol
    % 转弯
    radius = veh_params.wheel_base / tan(steer_angle);  %转弯半径
    center_x = x0 - radius * sin(theta0);
    center_y = y0 + radius * cos(theta0);
    
    delta_theta = delta_dist / radius;
    theta = theta0 + delta_theta;
    x = center_x + radius * sin(theta);
    y = center_y - radius * cos(theta);
else
    % 直行
    x = x0 + delta_dist * cos(theta0);
    y = y0 + delta_dist * sin(theta0);
    theta = theta0;
end

theta = mod(theta, 2*pi); %theta的范围为0~2*pi
veh_pose = [x, y, theta];


