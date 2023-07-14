#define t 5000

void setup() {
  pinMode(23, OUTPUT);
}

void loop() {
  digitalWrite(23, HIGH);
  delay(t);
  digitalWrite(23, LOW);
  delay(t);
}
