function  command=solve_mpc_problem(A, B, C,matrix_q,matrix_r,...
    lower, upper, ref_state, horizon, control,...
    matrix_state,control_state)
% MPC求解器 
% 输出:
% cmmand    : 输出命令 [steer cmd, acc]

% 输入:
% A         : 状态矩阵
% B         : 控制矩阵
% C         : 附加矩阵
% matrix_q  : 状态加权矩阵
% matrix_r  : 控制加权矩阵
% lower     ：控制下限
% upper     ：控制上限
% horizon   ：预测步数
% control   ：控制量数目
% matrix_state   : 输入状态矩阵
% control_state  : 输出控制状态矩阵


% discrete linear predictive control solver, with control
% format      x(i + 1) = A * x(i) + B * u (i) + C
% initial matrix
% matrix aa
q=size(A,1);
matrix_aa=zeros(size(A,1),horizon * size(A,2));
for i=1:horizon
    matrix_aa(1:q,(i-1)*q+1:(i-1)*q+q)=A^(i); %6*60
end


% matrix k  60*20
K_cell=cell(horizon,horizon);
for r=1:horizon
    for c=1:horizon
        if c<=r
        K_cell{r,c}=A^(r-c)*B;
        else
        K_cell{r,c}=zeros(size(B,1),size(B,2));
        end
    end
end
matrix_k=cell2mat(K_cell);

% matrix c
matrix_cc=zeros(horizon*size(C,1),size(C,2));
c_r=size(C,1);
matrix_cc(1:c_r,1)=C;
for j=2:horizon
     matrix_cc((j-1)*c_r+1:(j-1)*c_r+c_r,1)=A^(j-1)*C+...
         matrix_cc((j-2)*c_r+1:(j-2)*c_r+c_r,1);
end

% update matrix Q and R
matrix_Q=cell(horizon,horizon);
matrix_R=cell(horizon,horizon);
for i=1:horizon
    for j= 1:horizon
        if i == j
            matrix_Q{i,j}=matrix_q;
            matrix_R{i,j}=matrix_r;
        else 
            matrix_Q{i,j}=zeros(size(matrix_q,1),...
                size(matrix_q,2));
            matrix_R{i,j}=zeros(size(matrix_r,1),...
                size(matrix_r,2));
        end
    end
end
matrix_qq=cell2mat(matrix_Q); % 60*60
matrix_rr=cell2mat(matrix_R); % 20*20

% marix ll and matrix uu
matrix_LL=cell(horizon,1);
matrix_UU=cell(horizon,1);
for i=1:horizon
    matrix_LL{i,1}=lower;
    matrix_UU{i,1}=upper;
end
matrix_ll=cell2mat(matrix_LL); % 20*1
matrix_uu=cell2mat(matrix_UU); % 20*1

% matrix m  60*1
matrix_M=cell(horizon,1);
for i=1:horizon
    matrix_M{i,1}=A^(i)*matrix_state;
end
matrix_m=cell2mat(matrix_M);

% matrix t 60*1
matrix_t=zeros(size(B,1)*horizon,1); % reference state

% matrix v 20*1
matrix_V=cell(horizon,1); % control state
for i=1:horizon
    matrix_V{i,1}=control_state;
end
matrix_v=cell2mat(matrix_V);

% update matrix_m1, matrix_m2, convert MPC problem to
% QP problem
matrix_m1=matrix_k' * matrix_qq * matrix_k+matrix_rr; %size 20 20
matrix_m1=(matrix_m1+matrix_m1')/2; % make m1 symmatric 
matrix_m2=matrix_k' * matrix_qq * (matrix_m+matrix_cc...
    +matrix_t); % size 20*1

% Format in qp_solver
% min_x  : q(x) = 0.5 * x^T * Q * x  + x^T c
% with respect to:  A * x = b (equality constraint)
% C * x >= d (inequality constraint)
[l_row,l_col]=size(matrix_ll); %20
matrix_inequality_constrain_ll=eye(horizon*control,horizon*control);
u_row=size(matrix_uu,1); %20
matrix_inequality_constrain_uu=eye(horizon*control,1);

% solve QR problem
options=optimset('Algorithm','interior-point-convex',...
   'Display', 'off');
[matrix_v,fval,exutflag,message]=quadprog(matrix_m1,matrix_m2,...
    matrix_inequality_constrain_ll,...
    matrix_inequality_constrain_uu,...
    [],[],matrix_ll,matrix_uu,[],options);

% 选取算出的第一组量作为控制输入
command=matrix_v(1:2,1);
% pause

