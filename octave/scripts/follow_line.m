function thetas = follow_line(p_init, pf, n_points, Rd, vec_w, vec_q, N, R0, p0)
  points = line_points(p_init, pf, n_points);
  thetas = zeros(N, n_points);
  iterations_num = 0;
   
  MAX_ERRO_P = .005;% mm
  MAX_ERRO_R = pi/200;% rad

  for i = 1:n_points
    if (i == 1) theta_init = rand(N,1) * 2 * pi;
    else        theta_init = thetas(:,i-1);
    end

    it_num = 15;
    [thetas(:,i), erro_R, erro_p] = inv_kin(vec_w, vec_q, R0, p0, Rd, points(:,i), N, MAX_ERRO_R, MAX_ERRO_P, theta_init, it_num);

    if (erro_p > MAX_ERRO_P || erro_R > MAX_ERRO_R)
      [thetas(:,i), erro_R, erro_p] = inv_kin(vec_w, vec_q, R0, p0, Rd, points(:,i), N, MAX_ERRO_R, MAX_ERRO_P, thetas(:,i), 15);
      it_num += 15;

      if (erro_p > MAX_ERRO_P || erro_R > MAX_ERRO_R)
        printf("Erro of %i above max: %f %f\n", i, erro_p, erro_R);
      end
    end
    printf(" it: %i\n", it_num);
  end
endfunction
