function y = expmat_w(w, theta)
  W = [  0    -w(3)   w(2);
        w(3)    0    -w(1);
       -w(2)   w(1)     0  ];

  I = diag([1 1 1]);
  y = I + sin(theta) * W + (1 - cos(theta)) * W^2;
endfunction
