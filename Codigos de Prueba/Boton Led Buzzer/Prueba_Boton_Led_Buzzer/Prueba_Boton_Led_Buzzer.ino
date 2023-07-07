#define led 2
#define boton 13
#define buzzer 12

void setup() {
  pinMode(led, OUTPUT);
  pinMode(boton, INPUT_PULLUP);
  EasyBuzzer.setPin(buzzer);
}

void loop() {
  int estadoBoton;
  estadoBoton = digitalRead(boton);

  if (estadoBoton == LOW) {
    digitalWrite(led, HIGH);
    EasyBuzzer.beep(2000);
    EasyBuzzer.update();
  } else {
    digitalWrite(led, LOW);
    EasyBuzzer.stopBeep();
    EasyBuzzer.update();
  }
}
