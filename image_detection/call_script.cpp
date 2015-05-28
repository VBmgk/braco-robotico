#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>

using namespace std;
int mrks_num = 0,
    org = 0, ux = 1, uy = 2;

double mrks[3][2] = {{}, {}, {}},
       // c_norm is the u_norm in the camera ref
       u_norm = 40, cx_norm = 1, cy_norm = 1, vx[2], vy[2]; //mm


void transform(double p_on_camera[2], double p_on_ref[2]) {
  if(mrks_num != 3) {
    cout << "You must provide 3 red marks!!" << endl;
    return;
  }

  // v_p esc v_x = p_x
  double v_p[2] =
    { p_on_camera[0] - mrks[org][0],
      p_on_camera[1] - mrks[org][1] };
  
  p_on_ref[0] =  v_p[0] * vx[0] + v_p[1] * vx[1];
  p_on_ref[1] =  v_p[0] * vy[0] + v_p[1] * vy[1];

  p_on_ref[0] *= u_norm / cx_norm;
  p_on_ref[1] *= u_norm / cy_norm; 
}

void addToMarks(double cx, double cy) {
  if (mrks_num > 2) {
    cout << "marks list full" << endl;
    return;
  }

  mrks[mrks_num] [0] = cx; mrks[mrks_num] [1] = cy; 
  mrks_num++;
}

void order_marks () {
  if(mrks_num != 3) {
    cout << "You must provide 3 red marks!!" << endl;
    return;
  }

  // discover origin: point with less distance from others
  float d2_01 = (mrks[0][0] - mrks[1][0]) * (mrks[0][0] - mrks[1][0]) +
                (mrks[0][1] - mrks[1][1]) * (mrks[0][1] - mrks[1][1]),
        d2_02 = (mrks[0][0] - mrks[2][0]) * (mrks[0][0] - mrks[2][0]) +
                (mrks[0][1] - mrks[2][1]) * (mrks[0][1] - mrks[2][1]),
        d2_12 = (mrks[1][0] - mrks[2][0]) * (mrks[1][0] - mrks[2][0]) +
                (mrks[1][1] - mrks[2][1]) * (mrks[1][1] - mrks[2][1]);

  if      (d2_01 + d2_02 < d2_01 + d2_12 &&
           d2_01 + d2_02 < d2_02 + d2_12) { org = 0; }
  else if (d2_01 + d2_12 < d2_01 + d2_02 &&
           d2_01 + d2_12 < d2_02 + d2_12) { org = 1; }
  else { org = 2; }

  ux = (ux + org) % 3;
  uy = (uy + org) % 3;

  // discover ux and uy using signal of
  // last componet of vectorial product
  if (mrks[ux][0] * mrks[uy][1] - mrks[ux][1] * mrks[uy][0] < 0) {
    // change indexes
    int buff = ux; ux = uy; uy = buff; 
  }

  // Computing unitary vector to make
  // referential transformations
  vx[0] = mrks[ux][0] - mrks[org][0];
  vx[1] = mrks[ux][1] - mrks[org][1];

  vy[0] = mrks[uy][0] - mrks[org][0];
  vy[1] = mrks[uy][1] - mrks[org][1];

  // normalization
  cx_norm = sqrt(vx[0] * vx[0] + vx[1] * vx[1]);
  vx[0] /= cx_norm; vx[1] /= cx_norm;

  cy_norm = sqrt(vy[0] * vy[0] + vy[1] * vy[1]);
  vy[0] /= cy_norm; vy[1] /= cy_norm;
}

int main(){
  string command = "python image_detection.py";
  FILE* in = popen(command.c_str(), "r");
  
  char color_name[64];
  int cx, cy, img_h, img_w;
  double tx, ty;
  fscanf(in, "%d %d", &img_h, &img_w);
  cout << img_h << " " << img_w << endl;
  
  while(fscanf(in, "%s %d %d %lf %lf", color_name, &cx, &cy, &tx, &ty)!=EOF){
    cout << color_name << " " << cx << " " << cy
                       << " " << tx << " " << ty << endl;

    // Collecting data to covert points coordinates
    if (strcmp(color_name, "red") == 0) { addToMarks(cx, cy); }
  }

  order_marks();

  double p_on_mark_rf[2] = {};
  transform(mrks[ux] , p_on_mark_rf);
  cout << p_on_mark_rf[0] << " " << p_on_mark_rf[1] << endl;

  transform(mrks[uy] , p_on_mark_rf);
  cout << p_on_mark_rf[0] << " " << p_on_mark_rf[1] << endl;

  transform(mrks[org] , p_on_mark_rf);
  cout << p_on_mark_rf[0] << " " << p_on_mark_rf[1] << endl;
  
  pclose(in);
  
  return 0;
}
