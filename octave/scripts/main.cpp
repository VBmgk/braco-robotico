#define _USE_MATH_DEFINES
#include <cmath>
#include <tuple>
#include <array>
#include <algorithm>

#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <mat2cpp.hpp>

using namespace boost::numeric::ublas;
using namespace mat2cpp;
using namespace std;
using num = double;
using mat = matrix<num>;
using zeros = zero_matrix<num>;
using iden = identity_matrix<num>;
using vec = boost::numeric::ublas::vector<num>;

static constexpr num M_2PI = 2 * M_PI;

template <typename T> num norm(const T e) {
  return max(abs(max(e)), abs(min(e)));
}
num norm(const vec e) { return sqrt(inner_prod(e, e)); }

vec fix(const vec v) {
  vec nv(v.size());
  for (size_t i = 0; i < v.size(); i++) {
    nv(i) = floor(v(i));
  }
  return nv;
}

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

tuple<vec, num, num> inv_kin(const mat vec_w, const mat vec_q,
                             const mat R0, const vec p0, const mat Rd,
                             const vec pd, const vec t0) {
  static constexpr num TOL_R(M_PI_2 / 100), TOL_P(0.005), TOL(0.001);
  static constexpr size_t MAX_IT(100);
  static const size_t N = vec_w.size1();

  // initial theta values as a line vector
  // in the interval 0 - 2 pi
  // t_k = rand(N, 1) * 2 * pi;
  vec t_k = t0;

  // Jacobian matrix
  mat J = zeros(12, N);

  // column vector buffers
  vec z(12);
  mat f(12, 1);

  // set constants values
  num erro_r = 100;
  num erro_p = 100;
  size_t n_it = 1;

  // Main iteration loop
  while (n_it < MAX_IT && (erro_p > TOL_P || erro_r > TOL_R)) {
    n_it++;

    // f vector
    mat R_k;
    vec p_k;
    tie(R_k, p_k) = dir_kin(vec_w, vec_q, t_k, R0, p0);

    mat R_f = R_k - Rd;
    vec p_f = p_k - pd;

    f <<= R_f(0, 0), R_f(0, 1), R_f(0, 2), R_f(1, 0), R_f(1, 1),
        R_f(1, 2), R_f(2, 0), R_f(2, 1), R_f(2, 2), p_f(0), p_f(1),
        p_f(2);

    // Computation of Jacobian matrix and
    for (int i = 0; i < N; i++) {
      vec t_k_i_p(t_k);
      t_k_i_p(i) += TOL;

      vec t_k_i_n(t_k);
      t_k_i_n(i) -= TOL;

      // Jacobian
      // partial derivative
      //     delta     R^k
      // -------------     =
      // delta theta_i
      //
      //     R^k( theta^k + delta theta_i) + R^k( theta^k - delta
      //     theta_i)
      //    ---------------------------------------------------------------
      //                        2 tol

      mat R_k_i_p, R_k_i_n;
      vec p_k_i_p, p_k_i_n;
      tie(R_k_i_p, p_k_i_p) = dir_kin(vec_w, vec_q, t_k_i_p, R0, p0);
      tie(R_k_i_n, p_k_i_n) = dir_kin(vec_w, vec_q, t_k_i_n, R0, p0);

      mat delta_i_R_k = (R_k_i_p - R_k_i_n) / (2 * TOL);
      vec delta_i_p_k = (p_k_i_p - p_k_i_n) / (2 * TOL);

      J(0, i) = delta_i_R_k(0, 0);
      J(1, i) = delta_i_R_k(0, 1);
      J(2, i) = delta_i_R_k(0, 2);
      J(3, i) = delta_i_R_k(1, 0);
      J(4, i) = delta_i_R_k(1, 1);
      J(5, i) = delta_i_R_k(1, 2);
      J(6, i) = delta_i_R_k(2, 0);
      J(7, i) = delta_i_R_k(2, 1);
      J(8, i) = delta_i_R_k(2, 2);
      J(9, i) = delta_i_p_k(0);
      J(10, i) = delta_i_p_k(1);
      J(11, i) = delta_i_p_k(2);
    }

    // As the system may not have J square, it is necessary
    // to rearenge it:
    //
    // (J'.J).z = -J'.f
    // z = (prod(trans(J) * J)) \ (-prod(trans(J), f));
    size_t _r;
    z = column(matrix_div(prod(trans(J), J), prod(-trans(J), f), _r),
               0);

    t_k = z + t_k;

    // normalization [-2.pi, 2.pi]
    t_k = t_k - M_2PI * fix(t_k / M_2PI);

    // [0, 4.pi]
    t_k = t_k + M_2PI * scalar_vector<num>(t_k.size(), 1);

    // [0, 2.pi]
    t_k = t_k - M_2PI * fix(t_k / M_2PI);

    erro_r = norm_inf(R_f);
    erro_p = norm(p_f);
  }

  return make_tuple(t_k, erro_r, erro_p);
}

mat line_points(const vec p0, const vec pf, int n_points) {
  mat points = zeros(n_points, p0.size());

  for (int i = 0; i < n_points; i++)
    row(points, i) = p0 + i * (pf - p0) / (n_points - 1);

  return points;
}

mat follow_line(const vec p_init, const vec pf, int n_points,
                const mat Rd, const mat vec_w, const mat vec_q,
                const mat R0, const vec p0) {
  static constexpr num MAX_ERRO_R(M_PI_2 / 100), MAX_ERRO_P(0.005);
  static const size_t N = vec_w.size1();

  mat points = line_points(p_init, pf, n_points);
  mat thetas = zeros(n_points, N);

  for (int i = 0; i < n_points; i++) {
    vec theta_init;
    if (i == 0)
      theta_init = row(rand(1, N), 0) * M_2PI;
    else
      theta_init = row(thetas, i - 1);

    // inv_kin(const mat vec_w, const mat vec_q,
    //         const mat R0, const vec p0, const mat Rd,
    //         const vec pd, const vec t0) {

    num erro_r, erro_p;
    auto r = row(thetas, i);
    tie(r, erro_r, erro_p) =
        inv_kin(vec_w, vec_q, R0, p0, Rd, row(points, i), theta_init);

    int it = 100;
    if (erro_p > MAX_ERRO_P || erro_r > MAX_ERRO_R) {
      tie(r, erro_r, erro_p) =
        inv_kin(vec_w, vec_q, R0, p0, Rd, row(points, i), theta_init);
      it += 100;

      if (erro_p < MAX_ERRO_P || erro_r > MAX_ERRO_R) {
        cerr << "Error of " << i << " above max: " << erro_p << " " << erro_r << endl;
      }
    }

    //cout << " it: " << it << endl;
  }

  return thetas;
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

  vec t0(5);
  t0 <<= M_PI_2, 0, 0, 0, 0;

  cout << "w  = " << w << endl;
  cout << "q  = " << q << endl;
  cout << "R0 = " << R0 << endl;
  cout << "p0 = " << p0 << endl;
  cout << "θ  = " << t0 << endl;

#if 0
  mat R;
  vec p;
  tie(R, p) = dir_kin(w, q, t0, R0, p0);

  cout << "p  = " << p << std::endl;
  cout << "R  = " << R << std::endl;
#endif

#if 0
  vec t;
  num e0, e1;
  tie(t, e0, e1) = inv_kin(w, q, R0, p0, R0, p0, t0);

  cout << "t  = " << t << endl;
  cout << "e0 = " << e0 << endl;
  cout << "e1 = " << e1 << endl;
#endif

#if 0
  vec pi(3), pf(3);
  pi <<= 1, 0, 0;
  pf <<= 0, 1, 0;
  cout << line_points(pi, pf, 4) << endl;
#endif

#if 1
  vec p_init(3);
  p_init <<= 0, 100, 174;
  vec pf(3);
  pf <<= 0, 200, 174;
  cout << follow_line(p_init, pf, 10, R0, w, q, R0, p0) << endl;
#endif

  return 0;
}
