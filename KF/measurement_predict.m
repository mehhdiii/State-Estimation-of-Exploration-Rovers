function [yhat_last,K_k] = measurement_predict(xhat_k, H_k, P_k_last, R_k, b)

vx = xhat_k(4); 
omega = xhat_k(5);

yhat_last = [vx+b*omega; vx-b*omega]; 
S_k = H_k*P_k_last*H_k' + R_k; 
K_k = P_k_last*H_k'*inv(S_k); 

end

