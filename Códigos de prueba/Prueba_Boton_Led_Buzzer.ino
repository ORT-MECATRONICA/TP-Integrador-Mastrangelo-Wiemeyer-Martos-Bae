#define apretado 0
#define no_apretado 1
#define boton1 27 //33: abajo(funciona al revez), 32:atras , 25: arriba, 26: adelante , 27: aceptar
#define led1 14
#define led2 13
#define led3 12
#define buzzer 12
int estadoBoton1;

void setup() {
  pinMode(boton1, INPUT_PULLUP);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  estadoBoton1 = digitalRead(boton1);
  if (estadoBoton1 == apretado) {
    Serial.println("Boton 1 apretado");
    digitalWrite(buzzer, HIGH);
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
  } else {
    digitalWrite(buzzer, LOW);
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
  }
}
