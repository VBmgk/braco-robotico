#include <iostream>
#include <stdio.h>

using namespace std;

int main(){
	string command = "python image_detection.py";
	FILE* in = popen(command.c_str(), "r");

	char color_name[64];
	int cx, cy;
	double tx, ty;
	while(fscanf(in, "%s %d %d %lf %lf", color_name, &cx, &cy, &tx, &ty)!=EOF){
		cout << color_name << " " << cx << " " << cy << " " << tx << " " << ty << endl;
	}

	pclose(in);

	return 0;
}
