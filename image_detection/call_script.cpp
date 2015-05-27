#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace std;
int mrks_num = 0,
    org = 0, ux = 1, uy = 2, u_norm = 40; //mm
double mrks[3][2] = {{}, {}, {}};

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

  // show result (debug)
  cout << "x: " << mrks[ ux][0] << ',' << mrks[ ux][1] << endl
       << "y: " << mrks[ uy][0] << ',' << mrks[ uy][1] << endl
       << "o: " << mrks[org][0] << ',' << mrks[org][1] << endl;
}

int main(){
  string command = "python image_detection.py";
  FILE* in = popen(command.c_str(), "r");
  
  char color_name[64];
  int cx, cy, img_h, img_w;
  double tx, ty, real_w = 430, real_h = 260, px=0, py=0;
  fscanf(in, "%d %d", &img_h, &img_w);
  cout << img_h << " " << img_w << endl;
  
  while(fscanf(in, "%s %d %d %lf %lf", color_name, &cx, &cy, &tx, &ty)!=EOF){
    cout << color_name << " " << cx * real_w / img_w 
                         << " " << cy * real_h / img_h
                         << " " << tx << " " << ty << endl;

    // Collecting data to covert points coordinates
    if (strcmp(color_name, "red") == 0) { addToMarks(cx, cy); }
  }

  order_marks();
  
  pclose(in);
  
  return 0;
}
