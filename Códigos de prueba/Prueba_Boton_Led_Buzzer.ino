#define apretado 1
#define no_apretado 0
#define boton1 33
#define led1 14
#define buzzer 12
int estadoBoton1;

void setup() {
  pinMode(boton1, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  estadoBoton1 = digitalRead(boton1);
  if (estadoBoton1 == apretado) {
    Serial.println("Boton 1 apretado");
    digitalWrite(buzzer, HIGH);
    digitalWrite(led1, HIGH);
  } else {
    digitalWrite(buzzer, LOW);
    digitalWrite(led1, LOW);
  }
}
