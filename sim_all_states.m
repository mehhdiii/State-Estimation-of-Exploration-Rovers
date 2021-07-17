clear; close all; 
% define the system variables: 
T = 0.05;
% xk = f(xk-1)+vk = xk-1 + Tvkcos(phi{k-1}); yk-1 + Tvk sin(phi{k-1});
% phi{k-1} + T*wk




% define signal parameters: system
var_v1 =1e-6; 
var_v2 =1e-6; 
var_v3 = 1e-6; 
var_v4 = 1e-6; 
var_v5 = 1e-6; 
Q_k = diag([var_v1 var_v2 var_v3 var_v4 var_v5]); 


% define signal parameters: observation/sensor
var_w1 = 1e-8;
var_w2 = 1e-8; 
W_k = diag([var_w1 var_w2]); 
b = 0.005; %bias of odometer

ITER = 1e3; %number of iterations
x_last = [0; 0; 0; 5; 0.01]; %last value of states (X{k-1}): zero initially 
xhat_last = [0; 0; 0; 5; 0.01]; %Last optimal predicted value (X_hat{k-1}): zero initially 
P_last = eye(5); % Last covariance matrix value for the estimated states

y_last = 0; %last observation: zero initially 

%storage of values 
historyX_k = [0; 0; 0; 0; 0]; 
historyY_k = [0; 0];
historyX_predict = [0; 0; 0; 0; 0]; 

for k = 1:ITER   
    
    %generate noise values: 
    v = sqrt(Q_k)*randn(5, 1); 
    w = sqrt(W_k)*randn(2, 1); 
    
    %previous states:
    px = xhat_last(1); 
    py = xhat_last(2); 
    phi = xhat_last(3); 
    vx = xhat_last(4); 
    omega = xhat_last(5);
    
    %generate xk and yk: 
    x_k = [
        px + T*vx*cos(phi+T*omega/2); 
        py + T*vx*sin(phi+T*omega/2); 
        phi+T*omega; 
        vx;
        omega] + v; 
    
    y_k = [vx+b*omega; 
        vx-b*omega] + w;
    
    %run KF algorithm 
    [xhat_optimal,P_optimal] = KalmanFilter(y_k, Q_k, W_k, xhat_last, P_last, T, b); 
    xhat_last = xhat_optimal; 
    y_last = y_k; 
    x_last = x_k; 
    P_last = P_optimal; 
    historyX_k(:, k) = x_k; 
    historyY_k(:, k) = y_k; 
    historyX_predict(:, k) = xhat_optimal; 
end

P = 50
figure()
hold on 

plot(historyX_k(1, end-P:end), historyX_k(2, end-P:end), 'k-')
% plot(historyY_k(1, end-P:end).*cos(historyY_k(2, end-P:end)), historyY_k(1, end-P:end).*sin(historyY_k(2, end-P:end)), 'rx')
plot(historyX_predict(1, end-P:end), historyX_predict(2, end-P:end), 'b--', 'LineWidth',4)
xlabel("x coordinate", 'fontsize',12)
ylabel("y coordinate", 'fontsize',12)
title("Trajectory of Non-Linear system", 'fontsize',14)
lgd = legend('True trajectory', 'Predicted trajectory', 'location', 'best')
lgd.FontSize = 14
hold off
print -depsc results.eps