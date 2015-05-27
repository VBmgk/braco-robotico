function [R, p] = dir_kin(vec_w, vec_q, theta, N, R0, p0)
  % Guardando as origens para imprimir
  ps = zeros(3,2);  

  % Cinemática
  [R, p] = expmat_wv(vec_w(1,:), vec_q(1,:), theta(1));

  for n = 2:N
    [R_aux, p_aux] = expmat_wv(vec_w(n,:), vec_q(n,:), theta(n));

    p = R * p_aux + p;
    R = R * R_aux;
  end 
  
  p = R * p0' +  p;
  R = R * R0;
endfunction
