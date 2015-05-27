function points = line_points(p0, pf, n_points)
  % some magic conversions to make it work
  n_points = n_points - 1;

  points = zeros(3,n_points);

  for i = 1:n_points+1
    points(:,i) = p0 + (pf - p0) * ((i-1)/n_points);
  end
  %plot3(points(1,:), points(2,:), points(3,:));
endfunction
