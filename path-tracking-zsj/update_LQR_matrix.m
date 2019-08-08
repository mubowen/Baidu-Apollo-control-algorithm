function lqr_apollo_params=update_LQR_matrix(v,lqr_apollo_params)
% Upstate Matrix with velocity

 %v  ³µËÙ
 
 % Update the velocity of the Matrix A
 lqr_apollo_params.A(2,2)=lqr_apollo_params.A(2,2)/v;
 lqr_apollo_params.A(2,4)=lqr_apollo_params.A(2,3)/v;
 lqr_apollo_params.A(4,2)=lqr_apollo_params.A(4,2)/v;
 lqr_apollo_params.A(4,4)=lqr_apollo_params.A(4,4)/v;
 
