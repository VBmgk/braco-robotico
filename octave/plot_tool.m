function plot_tool(vec_w, vec_q, thetas, N, R0, p0, n_points)
  points = zeros(3,n_points);

  for n = 1:n_points
    [R, p] = dir_kin(vec_w, vec_q, thetas(:,n), N, R0, p0);
    points(:,n) = p;
  end

  plot3(points(1,:), points(2,:), points(3,:));
endfunction
