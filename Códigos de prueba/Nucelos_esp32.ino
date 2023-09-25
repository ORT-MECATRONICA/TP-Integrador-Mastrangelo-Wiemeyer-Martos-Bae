#define LED1 13
#define LED2 14
#define t1 500
#define t2 1600

TaskHandle_t tareaAlarma;

void setup() {
  Serial.print("Setup de corre en el nucleo: ");
  Serial.println(xPortGetCoreID()); //Para ver en que nucleo se ejecuta el setup
  
  xTaskCreatePinnedToCore(alarma, "alarma", 10000, NULL, 0, &tareaAlarma, 0);
  /*  xTaskCreatePinnedToCore(
      alarma, //nombre de la funcion de la tarea
      "alarma", //nombre de la tarea
      10000, //tamaño de pila
      NULL, //parametros de entrada
      0, //prioridad de la tarea (0 es la prioridad mas baja)
      &tareaAlarma, //objeto TaskHandle_t
      0); //nucleo donde se va a correr
  */

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop() {
  Serial.print("Loop de corre en el nucleo: ");
  Serial.println(xPortGetCoreID()); //Para ver en que nucleo se ejecuta el loop
  
  digitalWrite(LED1, HIGH);
  delay(t1);
  digitalWrite(LED1, LOW);
  delay(t1);
}

void alarma (void * pvParameters){
  Serial.print("Alarma de corre en el nucleo: ");
  Serial.println(xPortGetCoreID()); //Para ver en que nucleo se ejecuta la funcion alarma

  digitalWrite(LED2, HIGH);
  delay(t2);
  digitalWrite(LED2, LOW);
  delay(t2);
}
