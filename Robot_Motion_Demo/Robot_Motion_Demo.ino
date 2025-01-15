#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// #define servo_min 90
// #define servo_max 670
#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3
#define servo5 4
#define servo6 5
#define servo7 6
#define servo8 7
#define servo9 8

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

void Servo_2_Angle(char servo,int angle);

void Reset();
void Backward_2Legs();
void Move_Right();
void Move_Left();
void Forward_1Leg();
void Forward_3Legs();
void Rotate_Right();
void Rotate_Left();

bool flag = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(PB3, PB10); // PB3 SDA, PB10 SCL
  driver.begin();
  driver.setPWMFreq(50);
  Wire.setClock(400000);
      
  Reset();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Loop");
 Backward_2Legs();
}







void Servo_2_Angle(char servo,int angle) {
  int val;
  switch(servo) {
    case servo1:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo2:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo3:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo4:
      val = map(angle, 0, 180, 375, 2775); // 375, 2780
      break;

    case servo5:
      val = map(angle, 0, 180, 375, 2785);
      break;

    case servo6:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo7:
      val = map(angle, 0, 180, 375, 2770);
      break;

    case servo8:
      val = map(angle, 0, 180, 375, 2785);
      break;

    case servo9:
      val = map(angle, 0, 180, 375, 2780);
      break;
  }
  driver.writeMicroseconds(servo, val);
}

void Reset() {
//    for (int i = 0; i < 9; i++) {
    //  Servo_2_Angle(i, 0);
  // Servo_2_Angle(servo1, 45);
  // Servo_2_Angle(servo2, 45);
  // Servo_2_Angle(servo3, 67.5);
  // Servo_2_Angle(servo4, 45);
  // Servo_2_Angle(servo5, 67.5);
  // Servo_2_Angle(servo6, 45);
  // Servo_2_Angle(servo7, 67.5);
  // Servo_2_Angle(servo8, 45);
  // Servo_2_Angle(servo9, 45);
//    }
  driver.writeMicroseconds(0, 400);
  driver.writeMicroseconds(1, 400);
  driver.writeMicroseconds(2, 800);
  driver.writeMicroseconds(3, 400);
  driver.writeMicroseconds(4, 600);
  driver.writeMicroseconds(5, 400);
  driver.writeMicroseconds(6, 600);
  driver.writeMicroseconds(7, 400);
  driver.writeMicroseconds(8, 400);
    delay(1500);
}

void Backward_2Legs() {
  // driver.writeMicroseconds(0, 2400);
  // driver.writeMicroseconds(1, 400);
  // driver.writeMicroseconds(2, 1000);
  // driver.writeMicroseconds(3, 400);
  // driver.writeMicroseconds(4, 600);
  // driver.writeMicroseconds(5, 400);
  // driver.writeMicroseconds(6, 600);
  // driver.writeMicroseconds(7, 400);
  // driver.writeMicroseconds(8, 400);
  // delay(500);
  //trail 1 shohdy 14-1-2025
  // driver.writeMicroseconds(0, 2700);
  // driver.writeMicroseconds(1, 400);
  // driver.writeMicroseconds(2, 800);
  // driver.writeMicroseconds(3, 400);
  // driver.writeMicroseconds(4, 600);
  // driver.writeMicroseconds(5, 400);
  // driver.writeMicroseconds(6, 600);
  // driver.writeMicroseconds(7, 400);
  // driver.writeMicroseconds(8, 400);
  // delay(1500);
  //trail ووقع 14-1-2025
  // driver.writeMicroseconds(0, 2700);
  // driver.writeMicroseconds(1, 400);
  // driver.writeMicroseconds(2, 800);
  // driver.writeMicroseconds(3, 400);
  // driver.writeMicroseconds(4, 600);
  // driver.writeMicroseconds(5, 400);
  // driver.writeMicroseconds(6, 600);
  // driver.writeMicroseconds(7, 400);
  // driver.writeMicroseconds(8, 400);
  // delay(1500);
  Reset();
    // Servo_2_Angle(servo1, 160);
  // Servo_2_Angle(servo2, 45);
  // Servo_2_Angle(servo3, 110);
  // Servo_2_Angle(servo4, 45);
  // Servo_2_Angle(servo5, 67.5);
  // Servo_2_Angle(servo6, 45);
  // Servo_2_Angle(servo7, 67.5);
  // Servo_2_Angle(servo8, 45);
  // Servo_2_Angle(servo9, 45);
}

void Move_Lift(){
  Servo_2_Angle(servo8, 35);
  Servo_2_Angle(servo9, 35);
  Servo_2_Angle(servo5, 20);
  Servo_2_Angle(servo3, 20);

  Servo_2_Angle(servo4, 170);
  Servo_2_Angle(servo6, 170);
  Servo_2_Angle(servo2, 170);
  Servo_2_Angle(servo1, 170);
  delay(1500);
  Servo_2_Angle(servo4, 0);
  Servo_2_Angle(servo6, 0);
  Servo_2_Angle(servo2, 0);
  Servo_2_Angle(servo1, 0);
  delay(1500);
}

void Move_Right(){
  Servo_2_Angle(servo5, 35);
  Servo_2_Angle(servo6, 35);
  Servo_2_Angle(servo2, 20);
  Servo_2_Angle(servo9, 20);

  Servo_2_Angle(servo1, 170);
  Servo_2_Angle(servo3, 170);
  Servo_2_Angle(servo7, 170);
  Servo_2_Angle(servo8, 170);
  delay(1500);
  Servo_2_Angle(servo1, 0);
  Servo_2_Angle(servo3, 0);
  Servo_2_Angle(servo7, 0);
  Servo_2_Angle(servo8, 0);
  delay(1500);
}

void Forward_1Leg() {
  Servo_2_Angle(servo1, 170);
  delay(1000);
  Servo_2_Angle(servo1, 0);
  delay(1000);
}

void Forward_3Legs() {
  Servo_2_Angle(servo1, 170);
  delay(1500);
  Servo_2_Angle(servo6, 170);
  Servo_2_Angle(servo8, 170);
  delay(1500);
  Servo_2_Angle(servo1, 0);
  Servo_2_Angle(servo6, 0);
  Servo_2_Angle(servo8, 0);
  delay(1500);
}

void Rotate_Right() {
  Servo_2_Angle(servo7, 170);
  Servo_2_Angle(servo9, 170);

  Servo_2_Angle(servo1, 170);
  Servo_2_Angle(servo3, 170);

  Servo_2_Angle(servo4, 170);
  Servo_2_Angle(servo6, 170);
  
  delay(1500);
  Reset();
}

void Rotate_Left() {
  Servo_2_Angle(servo7, 170);
  Servo_2_Angle(servo8, 170);

  Servo_2_Angle(servo1, 170);
  Servo_2_Angle(servo2, 170);

  Servo_2_Angle(servo4, 170);
  Servo_2_Angle(servo5, 170);
  delay(1000);
  Reset();
}