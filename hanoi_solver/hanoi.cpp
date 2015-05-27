#include <iostream>
#include <string>
#include <vector>

#include "inv_kin.h"

#define DISC_HEIGHT 30 //mm
// TODO: measure
#define BASE_HEIGHT 20 //mm 
#define ALPHA .2

typedef enum Pin_pos {PIN_A, PIN_B, PIN_C} Pin_pos;

class Pin {
  Pin_pos position;
public:

  int num_discs;
  float x, y;

  Pin(Pin_pos p, float x, float y, int num):
    position(p), num_discs(num), x(x), y(y){}

  Pin_pos pos(void) { return position; }
  void remove() { num_discs--; }
  void add() { num_discs++; }
};

class Robot {
  Pin p_a, p_b, p_c;

public:
  Robot(Pin p_a, Pin p_b, Pin p_c):
    p_a(p_a), p_b(p_b), p_c(p_c) {
    // get pins positions
    openGripper();
  }
  
  ~Robot() { closeGripper(); }

  void send(std::string command) {
    std::cout << "@" + command + "@\n";
  }

  void goTo(float x, float y, float z) {
    std::vector<float> theta = {0,0,0,0,0};
    // inv kinematics
    solve(x,y,z, theta);

    std::string command =
      std::to_string(theta[0]) + " " +
      std::to_string(theta[1]) + " " +
      std::to_string(theta[2]) + " " +
      std::to_string(theta[3]) + " " +
      std::to_string(theta[4]);

    send(command);
  }

  void closeGripper(){ send("close"); }

  void openGripper(){ send("open"); }

  void removeFromPin (Pin& pin) {
    // get disc on the middle point 
    goTo(pin.x, pin.y, (pin.num_discs - 0.5) * DISC_HEIGHT + BASE_HEIGHT);
    // close Gripper!
    closeGripper();
    // go to the top of the pin
    goTo(pin.x, pin.y, (3 + ALPHA) * DISC_HEIGHT + BASE_HEIGHT);
    // update pin state
    pin.remove();
  }
  
  void moveToPin (Pin& pin) {
    // go to the top of the pin
    goTo(pin.x, pin.y, (3 + ALPHA) * DISC_HEIGHT + BASE_HEIGHT);
    // update pin state
    pin.add();
    // get disc on the middle point 
    goTo(pin.x, pin.y, (pin.num_discs - 0.5) * DISC_HEIGHT + BASE_HEIGHT);
    // open Gripper!
    openGripper();
  }
  
  void move_disc (Pin& orig, Pin& dest) {
    removeFromPin (orig);
    moveToPin (dest);
  } 
  
  void move_discs (Pin& orig, Pin& dest, int num_discs) {
    // Stop case of recorrence
    if (num_discs == 1) { move_disc (orig, dest); return; }
  
    // Discover intermediate pin
    if (orig.pos() != PIN_A && dest.pos() != PIN_A) {
      move_discs (orig, p_a, num_discs - 1);
      move_disc  (orig, dest);
      move_discs (p_a, dest, num_discs - 1);
    } else if (orig.pos() != PIN_B && dest.pos() != PIN_B) {
      move_discs (orig, p_b, num_discs - 1);
      move_disc  (orig, dest);
      move_discs (p_b, dest, num_discs - 1);
    } else {
      move_discs (orig, p_c, num_discs - 1);
      move_disc  (orig, dest);
      move_discs (p_c, dest, num_discs - 1);
    }
  }

  void solveHannoiTower() {
    move_discs (p_a, p_c, 3);
  }
};

int main (void) {
  Pin p_a(PIN_A, -70, 290, 3),
      p_b(PIN_B,   0, 290, 0),
      p_c(PIN_C,  70, 290, 0);
  
  Robot robot(p_a, p_b, p_c);

  robot.solveHannoiTower();
  
  return 0;
}
