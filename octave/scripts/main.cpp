#include "kin6dof.h"

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
