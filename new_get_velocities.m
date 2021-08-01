function [v,omega] = new_get_velocities(Xk, X_line, T, phi)
% Xk: Future input from lookup table or line tracking. 
% Xkm1: optimal estimate at given time. 




% Differential drive WMR Trajectory creator
xkm1 = Xk(1);
ykm1 = Xk(2);
xk = X_line(1); 
yk = X_line(2); 

%calculating inverse kinematics variables: 
mu = 1/2*(sin(phi)*(yk-ykm1)+cos(phi*(xk-xkm1)))...
    /(cos(phi)*(yk-ykm1)-sin(phi)*(xk-xkm1)); 
x_m = (xkm1+xk)/2; 
y_m = (ykm1+yk)/2; 

x_star = x_m - mu/2 * (yk - ykm1); 
y_star = y_m + mu/2 * (xk-xkm1); 

R_n = sqrt((xkm1 - x_star)^2 + (ykm1-y_star)^2); 
theta_1 = wrapToPi(atan2((ykm1-y_star), (xkm1-x_star))); 
theta_2 = wrapToPi(atan2((yk-y_star), (xk-x_star))); 
del_phi = wrapToPi(theta_1 - theta_2); 



%resulting Inv-Kinematics velocities: 
omega = del_phi/T; 
v = R_n*abs(omega);  

end

