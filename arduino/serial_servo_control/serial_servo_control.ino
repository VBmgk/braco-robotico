// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h>
 
Servo myservo[6];  // create servo object to control a servo 

int val[6] = {};
int income_byte;
int JOINT_NUM = 6;

boolean flag = false;

String pkg = "";


void setup() 
{ 
  // attaches the servos on pin 3,5,6,9,10 e 11
  myservo[0].attach(3); myservo[1].attach(5);
  myservo[2].attach(6); myservo[3].attach(9);
  myservo[4].attach(10); myservo[5].attach(11);
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

String readData(boolean& flag) {
  String buffer = "";
  income_byte = Serial.read();
  flag = false;
  
  // package = @data@
  if ((char) income_byte == '@') {
    flag = true;
    income_byte = Serial.read();
    
    while ((char) income_byte != '@') {
      // Discart trash
      if (income_byte != -1)
        buffer += (char) income_byte;
      
      income_byte = Serial.read();
    }
  }
  
  return buffer;
}

// pkg = @j<joint_num = 1 char>a<angle = 3 char>@
void decodePkg(){
  int aux[JOINT_NUM];
  if(flag == false) return;

  aux[0] = pkg.indexOf('a'); aux[1] = pkg.indexOf('b');
  aux[2] = pkg.indexOf('c'); aux[3] = pkg.indexOf('d');
  aux[4] = pkg.indexOf('e'); aux[5] = pkg.indexOf('f');
  
  // check if package is well formated  
  for(int i=0; i < JOINT_NUM ; i++){
    if(aux[i] == -1) { Serial.println("wrong package format"); return;}
  
    if(i < JOINT_NUM - 1)
         val[i] = pkg.substring(aux[i] + 1, aux[i+1]).toInt();
    else val[i] = pkg.substring(aux[i] + 1).toInt();
  }
  
  flag = true;
}

void setValues(){
  for (int i=0; i<JOINT_NUM ; i++)
    myservo[i].write(val[i]);
}

void loop() 
{ 
  // read package from serial port
  pkg = readData(flag);
  
  // decode package
  decodePkg();
  
  // sets the servo positions
  setValues();
  
  // waits for the servo to get there 
  delay(15);                           
} 
