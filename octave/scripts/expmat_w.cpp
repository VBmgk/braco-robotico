# include <mat2cpp.hpp>
# include <math.h>
# include <boost/numeric/ublas/io.hpp>

using namespace mat2cpp;

void expmat_w(const std::vector<double>& w,
              const double theta,
              matrix<double>& W, matrix<double>& R) {
  // Identity
  matrix<double> I(3,3); I(0,0) = I(1,1) = I(2,2) = 1;

  // w^
  W(0,0) =     0; W(0,1) = -w[2]; W(0,2) =  w[1];
  W(1,0) =  w[2]; W(1,1) =     0; W(1,2) = -w[0];
  W(2,0) = -w[1]; W(2,1) =  w[0]; W(2,2) =    0;

  // Rodrigues formula
  R = I + sin(theta) * W + (1 - cos(theta)) * prod(W, W);
}

void expmat_w(const std::vector<double>& w,
              const double theta, matrix<double>& R) {
  matrix<double> W(3,3);
  expmat_w(w, theta, W, R);
}

void expmat_wv (const std::vector<double>& _w,
                const std::vector<double>& _q,
                const double theta,
                matrix<double>& R, matrix<double>& p) {
  matrix<double> I(3,3); I(0,0) = I(1,1) = I(2,2) = 1;
  matrix<double> w(3,1), q(3,1), W(3,3);
  w(0,0) = _w[0]; w(1,0) = _w[1]; w(2,0) = _w[2];
  q(0,0) = _q[0]; q(1,0) = _q[1]; q(2,0) = _q[2];

  expmat_w(_w, theta, W, R);
  
  matrix<double> v = - prod(W, q);
  matrix<double> buff = prod(W, v),
                 buff2 = theta * prod(w, trans(w));

  // v = -W.q
  // p = (I - e^(w.t)). W . v + t . w . w' . v 
  p = prod((I - R), buff) + prod(buff2, v);
} 

// Fixed to 5 DOFs
void dir_kin (const std::vector< std::vector<double>>& vec_w,
              const std::vector< std::vector<double>>& vec_q,
              const std::vector<double>& theta,
              const matrix<double>&       R0,
              const std::vector<double>& _p0,
              matrix<double>&  R, matrix<double>&   p) {
  // Initial product
  // e^([w_1 q_1] * theta_1)
  expmat_wv (vec_w[0], vec_q[0], theta[0], R, p);

  matrix<double> R_aux(3,3), p_aux(3,1), p0(3,1);
  p0(0,0) = _p0[0]; p0(1,0) = _p0[1]; p0(2,0) = _p0[2];

  // Rest of the product
  // Prod e^([w_i q_i] * theta_i)
  for (int i=1; i<5 ;i++) {
    expmat_wv(vec_w[i], vec_q[i], theta[i], R_aux, p_aux);

    p = prod(R, p_aux) + p;
    R = prod(R, R_aux);
  } 
  
  p = prod(R, p0) +  p;
  R = prod(R, R0);
}

int main(void) {
	matrix<double> R(3, 3), p(3,1), R0(3,3);
        double h  = 93, l1 = 80, h2 = h + 81,
               l3 = 172 - l1;

        // initial orientation
        expmat_w({0,0,1}, 0, R0);

        std::vector<double> theta ({M_PI/2, 0, 0, 0, 0}),
                            p0 ({0, l3, h2});

        std::vector< std::vector<double>> w, q;

        w.push_back({ 0,0,1}); q.push_back({0,   0,  0});
        w.push_back({-1,0,0}); q.push_back({0,   0,  h});
        w.push_back({-1,0,0}); q.push_back({0, -l1,  h});
        w.push_back({-1,0,0}); q.push_back({0, -l1, h2});
        w.push_back({ 0,1,0}); q.push_back({0, -l1, h2});

        dir_kin(w, q, theta, R0, p0, R, p);

	std::cout << "p =" << p << std::endl;
	std::cout << "R =" << R << std::endl;

	return 0;
}
