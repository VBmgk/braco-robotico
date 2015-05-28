#include <iostream>
#include <string>
#include <vector>

#include "../octave/scripts/kin6dof.h"

#define DISC_HEIGHT 30 // mm
// TODO: measure
#define BASE_HEIGHT 20 // mm
#define ALPHA .2

typedef enum Pin_pos { PIN_A, PIN_B, PIN_C } Pin_pos;

static mat w(5, 3);
static mat q(5, 3);
static mat R0(3, 3);
static vec p0(3);
static vec t0(5);

void solve_init(void) {
  const num h = 93, l1 = 80, h2 = h + 81, l3 = 172 - l1;
  w <<= 0, 0, 1, -1, 0, 0, -1, 0, 0, -1, 0, 0, 0, 1, 0;
  q <<= 0, 0, 0, 0, 0, h, 0, -l1, h, 0, -l1, h2, 0, -l1, h2;
  R0 = iden(3);
  p0 <<= 0, l3, h2;
  t0 <<= M_PI_2, 0, 0, 0, 0;
}

std::vector<std::vector<float>> solve(float x, float y, float z) {
  static bool init = false;
  static float px, py, pz;

  std::vector<std::vector<float>> ts{};
  mat lines;
  vec p_init(3);
  vec pf(3);

  if (!init) {
    init = true;
    solve_init();
    goto over_and_out;
  }

  p_init <<= px, py, pz;
  pf <<= x, y, z;

  lines = follow_line(p_init, pf, 5, R0, w, q, R0, p0);
  for (int i = 0; i < lines.size1(); i++) {
    std::vector<float> t;
    for (int j = 0; j < 5; j++)
      t.push_back(lines(i, j));
    ts.push_back(t);
  }

over_and_out:
  px = x;
  py = y;
  pz = z;
  return ts;
}

class Pin {
  Pin_pos position;

public:
  int num_discs;
  float x, y;

  Pin(Pin_pos p, float x, float y, int num)
      : position(p), num_discs(num), x(x), y(y) {}

  Pin_pos pos(void) { return position; }
  void remove() { num_discs--; }
  void add() { num_discs++; }
};

class Robot {
  Pin p_a, p_b, p_c;

public:
  Robot(Pin p_a, Pin p_b, Pin p_c) : p_a(p_a), p_b(p_b), p_c(p_c) {
    // get pins positions
    openGripper();
  }

  ~Robot() { closeGripper(); }

  void send(std::string command) { std::cout << "@" + command + "@\n"; }

  void goTo(float x, float y, float z) {
    std::vector<std::vector<float>> thetas = solve(x, y, z);

    for (int i = 0; i < thetas.size(); i++) {
      auto theta = thetas[i];
      std::string command =
          std::to_string(theta[0]) + " " + std::to_string(theta[1]) +
          " " + std::to_string(theta[2]) + " " +
          std::to_string(theta[3]) + " " + std::to_string(theta[4]);
      send(command);
      // TODO: sleepmaybe???
    }
  }

  void closeGripper() { send("close"); }

  void openGripper() { send("open"); }

  void removeFromPin(Pin &pin) {
    // get disc on the middle point
    goTo(pin.x, pin.y,
         (pin.num_discs - 0.5) * DISC_HEIGHT + BASE_HEIGHT);
    // close Gripper!
    closeGripper();
    // go to the top of the pin
    goTo(pin.x, pin.y, (3 + ALPHA) * DISC_HEIGHT + BASE_HEIGHT);
    // update pin state
    pin.remove();
  }

  void moveToPin(Pin &pin) {
    // go to the top of the pin
    goTo(pin.x, pin.y, (3 + ALPHA) * DISC_HEIGHT + BASE_HEIGHT);
    // update pin state
    pin.add();
    // get disc on the middle point
    goTo(pin.x, pin.y,
         (pin.num_discs - 0.5) * DISC_HEIGHT + BASE_HEIGHT);
    // open Gripper!
    openGripper();
  }

  void move_disc(Pin &orig, Pin &dest) {
    removeFromPin(orig);
    moveToPin(dest);
  }

  void move_discs(Pin &orig, Pin &dest, int num_discs) {
    // Stop case of recorrence
    if (num_discs == 1) {
      move_disc(orig, dest);
      return;
    }

    // Discover intermediate pin
    if (orig.pos() != PIN_A && dest.pos() != PIN_A) {
      move_discs(orig, p_a, num_discs - 1);
      move_disc(orig, dest);
      move_discs(p_a, dest, num_discs - 1);
    } else if (orig.pos() != PIN_B && dest.pos() != PIN_B) {
      move_discs(orig, p_b, num_discs - 1);
      move_disc(orig, dest);
      move_discs(p_b, dest, num_discs - 1);
    } else {
      move_discs(orig, p_c, num_discs - 1);
      move_disc(orig, dest);
      move_discs(p_c, dest, num_discs - 1);
    }
  }

  void solveHannoiTower() { move_discs(p_a, p_c, 3); }
};

int main(void) {
  Pin p_a(PIN_A, -70, 290, 3), p_b(PIN_B, 0, 290, 0),
      p_c(PIN_C, 70, 290, 0);

  // TODO: call solve with initial position

  Robot robot(p_a, p_b, p_c);

  robot.solveHannoiTower();

  return 0;
}
