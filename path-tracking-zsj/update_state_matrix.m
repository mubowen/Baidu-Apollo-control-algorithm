function mpc_params=update_state_matrix(v,mpc_params)
% Upstate Matrix with velocity

 %v  ³µËÙ
 
 % Update the velocity of the Matrix A
 mpc_params.A(2,2)=mpc_params.A(2,2)/v;
 mpc_params.A(2,4)=mpc_params.A(2,3)/v;
 mpc_params.A(4,2)=mpc_params.A(4,2)/v;
 mpc_params.A(4,4)=mpc_params.A(4,4)/v;
 
 % update matrix C
 mpc_params.C(2,1)=(mpc_params.lr*mpc_params.cr-...
     mpc_params.lf*mpc_params.cf)/(mpc_params.mass*v)-v;
 mpc_params.C(4,1)=-(mpc_params.lf*mpc_params.lf*...
     mpc_params.cf+mpc_params.lr*mpc_params.lr*...
     mpc_params.cr)/(mpc_params.iz*v);