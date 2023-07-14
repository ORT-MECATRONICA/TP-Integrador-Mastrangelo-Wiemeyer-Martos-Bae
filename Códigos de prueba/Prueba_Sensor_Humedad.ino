//FUNCIONA
#define pinSensor 0
float valorSensor;

void setup() {
  Serial.begin(9600);
}

void loop() {
  valorSensor = map(analogRead(pinSensor), 0, 4096, 100, 0);
  Serial.print("Valor leído: ");
  Serial.print(valorSensor);
  Serial.println("%");
}
