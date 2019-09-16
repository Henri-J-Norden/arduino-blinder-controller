int i = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(11, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(11, 255);
  i += 10;
  if (i > 255) {
    i = 0;
  }
  delay(500);
  
}
