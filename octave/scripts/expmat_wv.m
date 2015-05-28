function [R, p] = expmat_wv(w, q, theta)
  I = diag([1 1 1]);

  W = [  0    -w(3)   w(2);
        w(3)    0    -w(1);
       -w(2)   w(1)     0  ];
  R = expmat_w(w, theta);

  v = - W * transpose(q);
  p = (I - R) * W * v + transpose(w) * w * v * theta;
endfunction
