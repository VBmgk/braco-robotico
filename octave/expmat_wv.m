function [R, p] = expmat_wv(w, q, theta)
  I = diag([1 1 1]);

  W = vectomat(w);
  R = expmat_w(w, theta);

  v = - W * transpose(q);
  p = (I - R) * W * v + transpose(w) * w * v * theta;
endfunction 
