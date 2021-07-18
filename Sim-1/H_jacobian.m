function H_k = H_jacobian(xhat_last, b)


H_k = [0 0 0 1 b; 0 0 0 1 -b];


end

