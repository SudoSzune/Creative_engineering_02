int pin = 7;
int sum = 0;

void setup() {
  pinMode(pin, OUTPUT);
}

void loop() {
  digitalWrite(7,0);
  delay(1000);
  while(sum<6){
    digitalWrite(7,0);
    delay(100);
    digitalWrite(7,1);
    delay(100);
    sum += 1;
  }
  while(1){
  digitalWrite(7,1);
  }
}
