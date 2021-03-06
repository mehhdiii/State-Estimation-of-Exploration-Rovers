function F_k = F_jacobian(T, xhat_last)
%Computes/updates jacobian matrix
px = xhat_last(1); 
py = xhat_last(2); 
phi = xhat_last(3); 
vx = xhat_last(4); 
omega = xhat_last(5);

F_k = [1 0 -T*vx*sin(phi+T*omega/2) T*cos(phi+T*omega/2) (-T*vx*sin(phi+T*omega/2))*T/2 ;
    0 1 T*vx*cos(phi+T*omega/2) T*sin(phi+T*omega/2) (T*vx*cos(phi+T*omega/2))*T/2;
    0 0 1 0 T; 
    0 0 0 1 0; 
    0 0 0 0 1]; 

end

