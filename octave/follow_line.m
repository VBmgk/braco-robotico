function thetas = follow_line(p_init, pf, n_points, Rd, vec_w, vec_q, N, R0, p0)
  points = line_points(p_init, pf, n_points);
  thetas = zeros(N, n_points);
   
  MAX_ERRO_P = .1;% mm
  MAX_ERRO_R = pi/200;% rad

  for i = 1:n_points
    [thetas(:,i), erro_R, erro_p] = inv_kin(vec_w, vec_q, R0, p0, Rd, points(:,i), N, MAX_ERRO_R, MAX_ERRO_P, zeros(N,1), 100);

    if (erro_p > MAX_ERRO_P || erro_R > MAX_ERRO_R)
      printf("Erro above max: %f %f", erro_p, erro_R);
    end
  end
endfunction
