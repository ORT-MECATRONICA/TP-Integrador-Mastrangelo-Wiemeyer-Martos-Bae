#define LDR_PIN 33
int valor;

void setup() {
  Serial.begin(9600);
  pinMode(LDR_PIN, INPUT);
}

void loop() {
  valor = analogRead(LDR_PIN);
  Serial.print("Valor leído = ");
  Serial.println(valor);
  delay(500);
}
