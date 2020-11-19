#include <Servo.h>
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

Servo myservo;
int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1500);


  Serial.begin(57600);

  a = 105;
  b = 390;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  if (dist_cali>255)
  {
    myservo.writeMicroseconds(1300);
  }
  else if(dist_cali<255)
  {
    myservo.writeMicroseconds(1700);
  }
  
  delay(20);
}
