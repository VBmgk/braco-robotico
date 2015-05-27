% Inverse kinematics using newton's numerical method
% for non-linear system of equations
% 
% Reference 
%   Title:  Métodos de Cãlculo Numérico
%   Author: José Paulo P. Dieguez

function [theta_k, erro_R, erro_p] = inv_kin(vec_w, vec_q, R0, p0, Rd, pd, N, TOL_R, TOL_pos, theta_0, MAX_IT)
  % angular diferential
  TOL = 0.001;

  % initial theta values as a line vector
  % in the interval 0 - 2 pi
  %theta_k = rand(N, 1) * 2 * pi;
  theta_k = theta_0;

  % Jacobian matrix
  J = zeros(12, N);
  
  % column vector buffers
  z = zeros(N,1);
  f = zeros(12,1);

  % set constants values
  erro_R = 100;
  erro_p = 100;
  n_it = 1;

  % Main iteration loop
  while (n_it < MAX_IT && (erro_p > TOL_pos || erro_R > TOL_R))
    n_it = 1 + n_it;

    % f vector
    [R_k, p_k] = dir_kin(vec_w, vec_q, theta_k, N, R0, p0);

    R_f = R_k - Rd;
    p_f = p_k - pd;

    f(1)  = R_f(1,1); f(2)  = R_f(1,2); f(3)  = R_f(1,3);
    f(4)  = R_f(2,1); f(5)  = R_f(2,2); f(6)  = R_f(2,3);
    f(7)  = R_f(3,1); f(8)  = R_f(3,2); f(9)  = R_f(3,3);
    f(10) = p_f(1)  ; f(11) = p_f(2)  ; f(12) = p_f(3);

    % Computation of Jacobian matrix and 
    for i = 1:N
      theta_k_i_p     = theta_k;
      theta_k_i_p(i)  = theta_k_i_p(i) + TOL;

      theta_k_i_n     = theta_k;
      theta_k_i_n(i)  = theta_k_i_n(i) - TOL;

      % Jacobian
      % partial derivative
      %     delta     R^k
      % -------------     = 
      % delta theta_i
      %
      %     R^k( theta^k + delta theta_i) + R^k( theta^k - delta theta_i)
      %    ---------------------------------------------------------------
      %                        2 tol
      
      [R_k_i_p, p_k_i_p] = dir_kin(vec_w, vec_q, theta_k_i_p, N, R0, p0);
      [R_k_i_n, p_k_i_n] = dir_kin(vec_w, vec_q, theta_k_i_n, N, R0, p0);
      
      delta_i_R_k = (R_k_i_p - R_k_i_n) / (2 * TOL);
      delta_i_p_k = (p_k_i_p - p_k_i_n) / (2 * TOL);

      J(1,i)  = delta_i_R_k (1,1); J(2,i)  = delta_i_R_k (1,2); J(3,i)  = delta_i_R_k (1,3);
      J(4,i)  = delta_i_R_k (2,1); J(5,i)  = delta_i_R_k (2,2); J(6,i)  = delta_i_R_k (2,3);
      J(7,i)  = delta_i_R_k (3,1); J(8,i)  = delta_i_R_k (3,2); J(9,i)  = delta_i_R_k (3,3);
      J(10,i) = delta_i_p_k (1);   J(11,i) = delta_i_p_k (2);   J(12,i) = delta_i_p_k (3);
    end
    
    % As the system may not have J square, it is necessary
    % to rearenge it:
    %
    % (J'.J).z = -J'.f
    z = (J' * J) \ (-J' * f);

    theta_k = z + theta_k;

    % normalization [-2.pi, 2.pi]
    theta_k = theta_k - 2 * pi * fix(theta_k / (2 * pi));

    % [0, 4.pi]
    theta_k = theta_k + 2 * pi;

    % [0, 2.pi]
    theta_k = theta_k - 2 * pi * fix(theta_k / (2 * pi));

    erro_R = norm(R_f,inf);
    erro_p = norm(p_f);
  endwhile
endfunction
