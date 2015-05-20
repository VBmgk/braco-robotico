function y = expmat_w(w, theta)
  W = vectomat(w);
  I = diag([1 1 1]);
  y = I + sin(theta) * W + (1 - cos(theta)) * W^2;
endfunction
