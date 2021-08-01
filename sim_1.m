clear; close all; clc;
% define the system variables: 


% define signal parameters: system
var_v1 =1e-3; 
var_v2 =1e-3; 
var_v3 = 1e-3;
var_v4 = 1e-3;
var_v5 = 1e-3;

Q_k = diag([var_v1 var_v2 var_v3 var_v4 var_v5]); 

% define signal parameters: observation/sensor
var_w1 = 1e-3;
var_w2 = 1e-3; 
W_k = diag([var_w1 var_w2]); 
b = 1/2; % Platform width = 2b

% Noise: 
var_d1 =1e-3;
var_d2 =1e-3; 
D_k = diag([var_d1 var_d2]); 

ITER = 10; %number of iterations
[x, y] = get_path(ITER);
T = 0.1;
% x_last = [0; 0; 0; 5; 0.01]; %last value of states (X{k-1}): zero initially 
xhat_last = [0; 0; 0; 0; 0]; %Last optimal predicted value (X_hat{k-1}): zero initially 
P_last = eye(5); % Last covariance matrix value for the estimated states

y_last = 0; %last observation: zero initially 

%storage of values 
historyX_k = [0; 0; 0; 0; 0]; 
historyY_k = [0; 0];
historyX_predict = [0; 0; 0; 0; 0]; 

for k = 2:ITER   
    
    %generate noise values: 
    v = sqrt(Q_k)*randn(5, 1); 
    w = sqrt(W_k)*randn(2, 1); 
    d = sqrt(D_k)*randn(2, 1); 
    %previous states:
    px = xhat_last(1); 
    py = xhat_last(2); 
%     phi = xhat_last(3) ;
    phi = wrapToPi(atan2(py-y(k-1), px-x(k-1))); 
    X_line = [x(k); y(k)] + d; 
    [vx,omega] = new_get_velocities(xhat_last, X_line, T, phi);
    disp([vx,omega]); drawnow
    
    %generate xk and yk: 
    x_k = [
        px + T*vx*cos(phi+T*omega/2); 
        py + T*vx*sin(phi+T*omega/2); 
        phi+T*omega; 
        vx;
        omega] + v; 
    
    y_k = [vx+b*omega; 
        vx-b*omega] + w;
    x_k(3) = wrapToPi(x_k(3));
    %run KF algorithm 
    [xhat_optimal,P_optimal] = KalmanFilter(y_k, Q_k, W_k, xhat_last, P_last, T, b); 
    xhat_optimal(3) = wrapToPi(xhat_optimal(3));
    xhat_last = xhat_optimal; 
    y_last = y_k; 
    x_last = x_k; 
    P_last = P_optimal; 
    historyX_k(:, k) = x_k; 
    historyY_k(:, k) = y_k; 
    historyX_predict(:, k) = xhat_optimal; 
end

P = ITER-1; 
figure()
hold on 

plot(x, y,'k-', 'LineWidth', 0.5 );
plot(historyX_k(1, :), historyX_k(2, :), 'ko:');
% plot(historyY_k(1, end-P:end).*cos(historyY_k(2, end-P:end)), historyY_k(1, end-P:end).*sin(historyY_k(2, end-P:end)), 'rx')
plot(historyX_predict(1, :), historyX_predict(2, :), 'b--', 'LineWidth',2);
xlabel("x coordinate", 'fontsize',12);
ylabel("y coordinate", 'fontsize',12);
title("Trajectory of Non-Linear system", 'fontsize',14);
lgd = legend('Ideal Path', 'Noisy trajectory', 'Predicted trajectory', 'location', 'best');
lgd.FontSize = 14;
hold off;
print -depsc results.eps;