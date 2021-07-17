function [xhat_k,P_k] = state_predict(xhat_last,Fk, P_last, Qk, T)
%Computes the mean and covariance of x_k|k-1
px = xhat_last(1); 
py = xhat_last(2); 
phi = xhat_last(3); 
vx = xhat_last(4); 
omega = xhat_last(5);

xhat_k = [px+T*vx*cos(phi+T*omega/2); 
    py+T*vx*sin(phi+T*omega/2);
    phi+T*omega;
    vx;
    omega];

P_k = Fk*P_last*Fk'+Qk;

end

