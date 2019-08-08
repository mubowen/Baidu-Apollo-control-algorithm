function steer_feedbackward = calc_lqr_Apollo_feedbackward(trajref,...
    delta_x, lqr_apollo_params, index, veh_params, veh_pose)
% 计算lqr的前轮偏角反馈控制量

% 输出:
% steer_feedbackward: 前轮偏角反馈控制量, rad

% 输入:
% trajref           : 期望路径[X, Y, Theta, Radius]
% delta_x           : 期望位姿与车辆当前位姿的偏差[dx, dy, dtheta]
% lqr_apollo_params        : LQR的参数
% index       : 滚动优化在trajref的初始index
% veh_params        : 车辆参数
% steer_feedforward : 前轮偏角前馈控制量, rad

% 1. 设置LQR的参数
delta_t = lqr_apollo_params.delta_t;   %LQR的时间步长, s
horizon = lqr_apollo_params.horizon;   %滚动优化窗口大小
Q = lqr_apollo_params.Q;               %状态误差权重矩阵
R = lqr_apollo_params.R;               %控制量权重矩阵
Q0 = lqr_apollo_params.Q0;             %状态误差终端矩阵

% 2. 设置车辆参数
v = veh_params.velocity;      %车速, m/s
wheel_base = veh_params.wheel_base;     %轴距, m

% 3. 将航向偏差限制到[-pi, pi]
delta_x(3) = angle_normalization(delta_x(3));

% 4. 设置Pk，Vk在终端状态时刻的值
Pk = Q0;
Vk = [0; 0; 0];

% 5. Update state
basic_state_size=4;
matrix_state=zeros(4,1);
control_state=zeros(4,1);

dx=delta_x(1);
dy=delta_x(2);
dtheta=delta_x(3);
theta_des=trajref(index,3);
theta=veh_pose(3);
radius_des=trajref(index,4);
k=trajref(index,5);  % curvature
one_min_k=1-k;
if one_min_k<=0
    one_min_k=0.01;
end
% 当前线速度
v=veh_params.velocity; 
% 期望速度
v_des=veh_params.v_des; 
% 当前转向角速度
angular_v=veh_params.angular_v;
% 期望转向角速度
angular_v_des=v_des*(1/radius_des);
if angular_v_des < 0.01
    angular_v_des=0;
end

%calculate the state
% lateral error
matrix_state(1,1)=dy*cos(theta_des)-dx*sin(theta_des);
% lateral error rate
matrix_state(2,1)=v*sin(dtheta);
% heading error
matrix_state(3,1)=angle_normalization(dtheta);
% heading error rate
heading_error_rate=angular_v-angular_v_des;
matrix_state(4,1)=heading_error_rate;

% Update Matrix
 lqr_apollo_params=update_LQR_matrix(v,lqr_apollo_params); 
 A=lqr_apollo_params.A;
 B=lqr_apollo_params.B;
 I=lqr_apollo_params.I;
 ts=lqr_apollo_params.ts;
 
 % 离散化matrix a
 % bilinear discrete matrix A
 matrix_ad=zeros(size(A));
 matrix_ad=(I+ts*0.5 * A) * inv(I-ts*0.5 * A); 
 
 % 离散化matrix b
 matrix_bd=zeros(size(B));
 matrix_bd=B*ts;

% 6. 滚动优化
 K = solve_lqr_problem(matrix_ad, matrix_bd,...
      Q, R, horizon);
  
% 7. 求得lqr的前轮偏角反馈控制量
 steer_feedbackward = -(K * matrix_state);

 




