#define _USE_MATH_DEFINES
#include <cmath>
#include <tuple>
#include <array>
#include <algorithm>

#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <mat2cpp.hpp>

using namespace boost::numeric::ublas;
using namespace mat2cpp;
using namespace std;
using num = float;
using mat = matrix<num>;
using zeros = zero_matrix<num>;
using iden = identity_matrix<num>;
using vec = boost::numeric::ublas::vector<num>;

mat expmat_w(const vec w, const num t) {

  // Identity
  mat I = iden(3);

  // w^
  mat W(3, 3);
  W <<= 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  // Rodrigues formula
  mat y = I + sin(t) * W + (1 - cos(t)) * prod(W, W);

  return y;
}

tuple<mat, vec> expmat_wv(const vec w, const vec q, const num t) {
  mat I = zeros(3, 3);
  I(0, 0) = I(1, 1) = I(2, 2) = 1;

  // w^
  mat W(3, 3);
  W <<= 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  auto R = expmat_w(w, t);

  // v = -W.q
  // p = (I - e^(w.t)). W . v + t . w . w' . v
  vec v = -prod(W, q);
  vec p = prod(prod(I - R, W), v) + t * w * inner_prod(w, v);
  ;

  return make_tuple(R, p);
}

// Fixed to 5 DOFs
tuple<mat, vec> dir_kin(const mat vec_w, const mat vec_q,
                        const vec theta, const mat R0, const vec p0) {

  // Initial product
  // e^([w_1 q_1] * theta_1)
  mat R;
  vec p;
  tie(R, p) = expmat_wv(row(vec_w, 0), row(vec_q, 0), theta(0));

  //// Rest of the product
  //// Prod e^([w_i q_i] * theta_i)
  for (int i = 1; i < 5; i++) {
    mat R_aux;
    vec p_aux;
    tie(R_aux, p_aux) =
        expmat_wv(row(vec_w, i), row(vec_q, i), theta(i));

    p = prod(R, p_aux) + p;
    R = prod(R, R_aux);
  }

  p = prod(R, p0) + p;
  R = prod(R, R0);

  return make_tuple(R, p);
}

tuple<vec, num, num> inv_kin(const mat vec_w, const mat vec_q, const mat R0,
                             const vec p0, const mat Rd, const vec pd,
                             const vec t0) {
  static constexpr num TOL_R(M_PI_2 / 100), TOL_P(0.005), TOL(0.001);
  static constexpr size_t MAX_IT(15);
  static const size_t N = vec_w.size1();

  // initial theta values as a line vector
  // in the interval 0 - 2 pi
  //theta_k = rand(N, 1) * 2 * pi;
  vec theta_k = t0;

  // Jacobian matrix
  mat J(12, N);

  // column vector buffers
  vec z(12), f(12);

  // set constants values
  num erro_r = 1;
  num erro_p = 1;
  size_t n_it = 1;

  // Main iteration loop
  while (n_it < MAX_IT && (erro_p > TOL_P || erro_r > TOL_R)) {
    n_it++;

  }

  return make_tuple(theta_k, erro_r, erro_p);
}

int main(void) {
  const num h = 93, l1 = 80, h2 = h + 81, l3 = 172 - l1;

  mat _w(5, 3);
  _w <<= 0, 0, 1, -1, 0, 0, -1, 0, 0, -1, 0, 0, 0, 1, 0;
  const mat w(_w);

  mat _q(5, 3);
  _q <<= 0, 0, 0, 0, 0, h, 0, -l1, h, 0, -l1, h2, 0, -l1, h2;
  const mat q(_q);

  const mat R0 = iden(3);

  vec _p0(3);
  _p0 <<= 0, l3, h2;
  const vec p0(_p0);

  vec t(5);
  t <<= M_PI_2, 0, 0, 0, 0;

  cout << "w  = " << w << endl;
  cout << "q  = " << q << endl;
  cout << "R0 = " << R0 << endl;
  cout << "p0 = " << p0 << endl;
  cout << "θ  = " << t << endl;

  mat R;
  vec p;
  tie(R, p) = dir_kin(w, q, t, R0, p0);

  cout << "p  = " << p << std::endl;
  cout << "R  = " << R << std::endl;

  return 0;
}
