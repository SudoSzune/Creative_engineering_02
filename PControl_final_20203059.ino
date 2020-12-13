#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.1

#define _DUTY_MIN 520 // 1300  1.315 105
#define _DUTY_NEU 1500 // 1650
#define _DUTY_MAX 2480 // 2000

#define _SERVO_ANGLE 600
#define _SERVO_SPEED 2000

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100
#define DELAY_MICROS 1500

#define _KP 1.1
#define _KD 0
Servo myservo;

float dist_target;
float dist_raw;

unsigned long last_sampling_time_dist, last_sampling_time_servo,last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr, dist_cali, dist_ema;

float error_curr, error_prev, control, pterm, dterm;
float duty_neutral;

float ema_dist=0;           
float filtered_dist;
float samples_num = 3; 

const float coE[] = {-0.0000139, 0.0084009, -0.2027187, 80.2882875};

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  duty_target, duty_curr = _DIST_MIN;
  last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;
  
  pterm = 0;
  dterm = 0;
  error_prev = 0;
  duty_neutral = _DUTY_NEU;
//  while(1){
//    myservo.writeMicroseconds(1700);
//  }
  Serial.begin(57600);
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
}
  

void loop() {
    unsigned long time_curr = millis();
    
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    if(event_dist) {
        event_dist = false;
        float x = filtered_ir_distance(); // [3037]
        float dist_cali = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
        dist_ema = dist_cali;
        
        error_curr = _DIST_TARGET - dist_ema;

        pterm = _KP * error_curr;
        dterm = _KD * (error_curr - error_prev);
        control = pterm + dterm;
        duty_target = duty_neutral + control;
        // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
        if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
        if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit
        // update error_prev
        error_prev = error_curr;
    }
  
    if(event_servo) {
      event_servo = false;
      
      if(duty_target > duty_curr) {
              duty_curr += duty_chg_per_interval;
              if(duty_curr > duty_target) duty_curr = duty_target;
      }
      else {
          duty_curr -= duty_chg_per_interval;
          if(duty_curr < duty_target) duty_curr = duty_target;
      }

      myservo.writeMicroseconds(duty_curr);
    }
  
    if(event_serial) {
        event_serial = false;
        Serial.print("dist_ir:");
        Serial.print(dist_ema);
        Serial.print(",pterm:");
        Serial.print(map(pterm,-1000,1000,510,610));
        Serial.print(",duty_target:");
        Serial.print(map(duty_target,1000,2000,410,510));
        Serial.print(",duty_curr:");
        Serial.print(map(duty_curr,1000,2000,410,510));
        Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    }
}

float ir_distance(void){ // return value unit: mm
    float val;
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0;
    return val;
}

float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){
  
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  
  ema_dist = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*ema_dist;
  return ema_dist;
}
