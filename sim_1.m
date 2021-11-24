 
clear; close all; clc;
% define the system variables:
ITER = 2000; %number of iterations
T = 0.1; % Sampling time
b = 0.4; % Platform width = 2b


% define signal parameters: system
var_v1 = 1e-4;
var_v2 = 1e-4;
var_v3 = 1e-6;
var_v4 = 1e-6;
var_v5 = 1e-6;


Q_k = diag([var_v1 var_v2 var_v3 var_v4 var_v5]);


% define signal parameters: observation/sensor
var_w1 = 1e-4;
var_w2 = 1e-4;
W_k = diag([var_w1 var_w2]);


x_k = [0;0;0;1;0]; % Plant initial state
xhat_last = [0;0;0;0;0]; % Last optimal predicted value (X_hat{​​​​​​​k-1}​​​​​​​): zero initially
P_last = eye(5); % Last covariance matrix value for the estimated states


%storage of values
historyX_k = zeros(5,1);
historyY_k = zeros(2,1);
historyX_pred = zeros(5,1);


for k = 2:ITER
    
    %generate noise values:
    v = sqrt(Q_k)*randn(5,1);
    w = sqrt(W_k)*randn(2,1);
    %generate xk and yk:
    vx = x_k(4);
    omega = x_k(5);
    phi = x_k(3);
    x_k = x_k + [
        T*vx*cos(phi+T*omega/2);
        T*vx*sin(phi+T*omega/2);
        T*omega;
        0;
        0] + v;
    x_k(3) = wrapToPi(x_k(3));
    y_k = [0 0 0 1 b; 0 0 0 1 -b]*x_k + w;
    
    %run KF algorithm
    [x_optimal,P_optimal] = ...
        KalmanFilter(y_k,Q_k,W_k,xhat_last,P_last,T,b);
    x_optimal(3) = wrapToPi(x_optimal(3));
    
    xhat_last = x_optimal;
    P_last = P_optimal;
    
    historyX_k(:,k) = x_k;
    historyY_k(:,k) = y_k;
    historyX_pred(:, k) = x_optimal;
end


figure();hold on
plot(historyX_k(1,:), historyX_k(2,:), 'kd:');
plot(historyX_pred(1,:), historyX_pred(2,:), 'bs--', 'LineWidth',2);
xlabel('x coordinate', 'fontsize',12);
ylabel('y coordinate', 'fontsize',12);
title('Comparing Trajectories', 'fontsize',14);
lgd = legend('Stochastic trajectory',...
    'Estimated trajectory', 'location', 'best');lgd.FontSize = 14;


figure; plot(historyY_k(1,2:end),'k--');hold on
plot(historyX_k(4,2:end)+b*historyX_k(5,2:end),'r-','linewidth',2)
plot(historyX_pred(4,2:end)+b*historyX_pred(5,2:end),'b-','linewidth',2)
title('Left-wheel Velocity unit/sec', 'fontsize',14);
lgd = legend('Noisy observation','True Values','Estimated Values',...
    'location', 'best');lgd.FontSize = 14;


figure; plot(historyY_k(2,2:end),'k--');hold on
plot(historyX_k(4,2:end)-b*historyX_k(5,2:end),'r-','linewidth',2)
plot(historyX_pred(4,2:end)-b*historyX_pred(5,2:end),'b-','linewidth',2)
title('Right-wheel Velocity unit/sec', 'fontsize',14);
lgd = legend('Noisy observation','True values','Estimated values',...
    'location', 'best');lgd.FontSize = 14;










