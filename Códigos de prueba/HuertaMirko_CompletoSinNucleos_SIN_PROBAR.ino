// Bae, Martos, Matrangelo, Wiemeyer

//Sensor temperatura
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;  // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

//Telegram
/*#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
const char* ssid = "China-Tati"; //"ORT-IoT";
const char* password = "mardelaspampas"//"OrtIOTnew22$2";
#define BOTtoken "6011340543:AAHn3L9_PeL9hfO5barxWzuc3tqZZbR3AhU"
#define CHAT_ID "-807022472"
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);*/

//Display
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

//BOTONES
#define ABAJO 32
#define ARRIBA 27
#define DERECHA 25
#define IZQUIERDA 26
#define BOTON_OK 33  ////////// Pin 33 tiene al reves el pull up
int estadoBotonDerecha;
int estadoBotonIzquierda;
int estadoBotonOk;
boolean flagBotonDerecha = 1;
boolean flagBotonIzquierda = 1;
boolean flagBotonArriba = 1;
boolean flagBotonAbajo = 1;
boolean flagBotonOk = 1;
#define presionado 0
#define no_presionado 1

boolean BotonRestaApretado;
#define si 1
#define no 0

int uno = 1;
int menosUno = -1;
int sesenta = 60;
int tresMil = 3000;

//LDR
#define PIN_LDR 35
int lecturaLDR;

//HUMEDAD
#define PIN_HUMEDAD 34
float lecturaHumedad;

//MAQUINA DE ESTADOS
int contador = 0;
int estado = 0;
#define pruebaBotones 22
#define MENU 0
#define CAMBIO_TEMP 1
#define CAMBIO_HUM 2
#define CAMBIO_GMT 3
#define CAMBIO_MQTT 4
int VALOR_UMBRAL_TEMP = 26;
int VALOR_UMBRAL_HUM;
int VALOR_GMT;
int VALOR_MQTT;

//Funcion alarma con temperatura, led, buzzer y cooler
#define LED_VERDE 14
#define LED_AMARILLO 12
#define LED_ROJO 13
#define BUZZER 17
#define COOLER 16
int lecturaTemperatura;

int maquinaEstadoAlarma;
#define VERDE 0
#define AMARILLO 1
#define ROJO 2
#define ROJO_SIN_BUZZER 3

//Nucleos
/*TaskHandle_t tareaAlarma;
TaskHandle_t tareaSensores;*/

void setup() {
  Serial.begin(9600);
  lcd.init();  //lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);

  pinMode(ABAJO, INPUT_PULLUP);
  pinMode(ARRIBA, INPUT_PULLUP);
  pinMode(DERECHA, INPUT_PULLUP);
  pinMode(IZQUIERDA, INPUT_PULLUP);
  pinMode(BOTON_OK, INPUT_PULLUP);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AMARILLO, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(COOLER, OUTPUT);

  pinMode(PIN_HUMEDAD, INPUT);
  pinMode(PIN_LDR, INPUT);

  // Temperatura y telegram
  while (!Serial) delay(100);  // wait for native usb
  Serial.println(F("BMP280 Sensor event test"));

  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  //status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  // Connect to Wi-Fi
  /*WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  // Bloquea el programa si no se puede conectar a internet
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }*/
  // Print ESP32 Local IP Address
  /*Serial.println(WiFi.localIP());
  bot.sendMessage(CHAT_ID, "Bot Hola mundo", "");*/

  //xTaskCreatePinnedToCore(alarma, "alarma", 10000, NULL, 0, &tareaAlarma, 0);

  //xTaskCreatePinnedToCore(sensores, "sensores", 10000, NULL, 0, &tareaSensores, 1);
}

void loop() {
  sensors_event_t temp_event;
  bmp_temp->getEvent(&temp_event);
  lecturaTemperatura = temp_event.temperature;
  Serial.println(lecturaTemperatura);

  estadoBotonOk = digitalRead(BOTON_OK);

  switch (estado) {
    case VERDE:
      Serial.println("Estado verde");
      digitalWrite(LED_VERDE, HIGH);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_ROJO, LOW);
      digitalWrite(BUZZER, LOW);
      digitalWrite(COOLER, HIGH);
      if (lecturaTemperatura >= VALOR_UMBRAL_TEMP) {
        estado = ROJO;
      }
      break;

    case AMARILLO:
      Serial.println("Estado amarillo");
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_AMARILLO, HIGH);
      digitalWrite(LED_ROJO, LOW);
      digitalWrite(BUZZER, LOW);
      digitalWrite(COOLER, HIGH);
      if (estadoBotonOk == no_presionado) {  //es al revez en este boton
        estado = VERDE;
      }
      if (lecturaTemperatura >= VALOR_UMBRAL_TEMP) {
        estado = ROJO;
      }
      break;

    case ROJO:
      Serial.println("Estado rojo");
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_ROJO, HIGH);
      digitalWrite(BUZZER, HIGH);
      digitalWrite(COOLER, LOW);
      //bot.sendMessage(CHAT_ID, "La temperatura ha superado el umbral establecido", "");
      if (estadoBotonOk == no_presionado) {  //es al revez en este boton
        estado = ROJO_SIN_BUZZER;
      }
      if (lecturaTemperatura < VALOR_UMBRAL_TEMP) {
        estado = AMARILLO;
      }
      break;

    case ROJO_SIN_BUZZER:
      Serial.println("Estado rojo sin buzzer");
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_ROJO, HIGH);
      digitalWrite(BUZZER, LOW);
      digitalWrite(COOLER, LOW);
      if (lecturaTemperatura < VALOR_UMBRAL_TEMP) {
        estado = AMARILLO;
      }
      break;
  }


  switch (estado) {
    case pruebaBotones:
      if (digitalRead(DERECHA) == LOW) {
        Serial.println("DERECHA");
      }
      if (digitalRead(IZQUIERDA) == LOW) {
        Serial.println("IZQUIERDA");
      }
      if (digitalRead(ABAJO) == LOW) {
        Serial.println("ABAJO");
      }
      if (digitalRead(ARRIBA) == LOW) {
        Serial.println("ARRIBA");
      }
      if (digitalRead(BOTON_OK) == HIGH) {
        Serial.println("BOTON_OK");
      }
      //      lecturaBotonesMenu ();
      break;

    case MENU:  //Pantalla 0: Para ver los valores leidos de temperatura, humedad y luz
      //Print del display
      //Serial.println("Modo: MENU");
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.setCursor(5, 0);
      char cadenaTemperatura[3];
      sprintf(cadenaTemperatura, "%03d", lecturaTemperatura);
      lcd.print(cadenaTemperatura);
      lcd.setCursor(9, 0);
      lcd.print("Hum:");
      lcd.setCursor(13, 0);
      lcd.print(lecturaHumedad);
      lcd.setCursor(0, 1);
      lcd.print("Luz:");
      lcd.setCursor(4, 1);
      char cadenaLDR[3];
      sprintf(cadenaLDR, "%03d", lecturaLDR);
      lcd.print(cadenaLDR);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu();
      if (contador == 1) {
        Serial.println("Entrando a modo CAMBIO TEMP");
        lcd.clear();
        estado = CAMBIO_TEMP;
      }

      //Lo que tiene que suceder en este estado - Mostrar los valores leídos de temperatura, humedad y luz
      lecturaHumedad = map(analogRead(PIN_HUMEDAD), 0, 4096, 100, 0);
      lecturaLDR = map(analogRead(PIN_LDR), 0, 4096, 0, 100);

      break;

    case CAMBIO_TEMP:  //Pantalla 1: Para cambiar el valor umbral de temperatura
      //Print del display
      //Serial.print("Modo: CAMBIO_TEMP - ");
      lcd.setCursor(0, 0);
      lcd.print("MODO CAMBIO_TEMP");
      lcd.setCursor(0, 1);
      lcd.print("Valor: ");
      char cadena[5];
      sprintf(cadena, "%02d", VALOR_UMBRAL_TEMP);
      lcd.print(cadena);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu();
      if (contador == 0) {
        lcd.clear();
        estado = MENU;
      }
      if (contador == 2) {
        lcd.clear();
        estado = CAMBIO_HUM;
      }

      //Lo que tiene que suceder en este estado
      VALOR_UMBRAL_TEMP = lecturaBotonAbajo(VALOR_UMBRAL_TEMP, uno);
      VALOR_UMBRAL_TEMP = lecturaBotonArriba(VALOR_UMBRAL_TEMP, uno);
      Serial.print("Valor TEMP: ");
      Serial.println(VALOR_UMBRAL_TEMP);

      //Serial.print(" - Contador: ");
      //Serial.println(contador);

      break;

    case CAMBIO_HUM:  //Pantalla 2: Para cambiar el valor umbral de hunedad
      //Print del display
      //Serial.print("Modo: CAMBIO_HUM - ");
      lcd.setCursor(0, 0);
      lcd.print("MODO CAMBIO_HUM");
      lcd.setCursor(0, 1);
      lcd.print("Valor: ");
      char cadena1[5];
      sprintf(cadena1, "%02d", VALOR_UMBRAL_HUM);
      lcd.print(cadena);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu();
      if (contador == 1) {
        lcd.clear();
        estado = CAMBIO_TEMP;
      }
      if (contador == 3) {
        lcd.clear();
        estado = CAMBIO_GMT;
      }

      //Lo que tiene que suceder en este estado
      VALOR_UMBRAL_TEMP = lecturaBotonAbajo(VALOR_UMBRAL_HUM, uno);
      VALOR_UMBRAL_TEMP = lecturaBotonArriba(VALOR_UMBRAL_HUM, uno);
      Serial.print("Valor HUM: ");
      Serial.println(VALOR_UMBRAL_HUM);

      //Serial.print(" - Contador: ");
      //Serial.println(contador);

      break;

    case CAMBIO_GMT:  //Pantalla 3: Para cambiar el valor de gmt
      //Print del display
      Serial.print("Modo: CAMBIO_GMT - ");
      lcd.setCursor(0, 0);
      lcd.print("MODO CAMBIO_GMT");
      lcd.setCursor(0, 1);
      lcd.print("Hora: ");
      char cadena2[5];
      sprintf(cadena2, "%05d", VALOR_GMT);
      lcd.print(cadena2);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu();
      if (contador == 2) {
        lcd.clear();
        estado = CAMBIO_TEMP;
      }
      if (contador == 4) {
        lcd.clear();
        estado = CAMBIO_MQTT;
      }

      //Lo que tiene que suceder en este estado
      VALOR_GMT = lecturaBotonAbajo(VALOR_GMT, tresMil);
      VALOR_GMT = lecturaBotonArriba(VALOR_GMT, tresMil);
      Serial.print("Hora GMT: ");
      Serial.print(VALOR_GMT);

      Serial.print(" - Contador: ");
      Serial.println(contador);

      break;

    case CAMBIO_MQTT:  //Pantalla 4: Para cambiar el tiempo de mqtt
      //Print del display
      Serial.print("Modo: CAMBIO_MQTT - ");
      lcd.setCursor(0, 0);
      lcd.print("MODO CAMBIO_MQTT");
      lcd.setCursor(0, 1);
      lcd.print("Valor: ");
      char cadena3[5];
      sprintf(cadena3, "%04d", VALOR_MQTT);
      lcd.print(cadena3);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu();
      if (contador == 2) {
        lcd.clear();
        estado = CAMBIO_GMT;
      }

      //Lo que tiene que suceder en este estado
      VALOR_MQTT = lecturaBotonAbajo(VALOR_MQTT, sesenta);
      VALOR_MQTT = lecturaBotonArriba(VALOR_MQTT, sesenta);
      if (VALOR_MQTT < 0) {
        VALOR_MQTT = 0;
      }
      Serial.print("Valor MQTT: ");
      Serial.print(VALOR_MQTT);

      Serial.print(" - Contador: ");
      Serial.println(contador);

      break;
  }
}

int lecturaBotonAbajo(int variableBotonAbajo, int cantidadResta) {
  int estadoBotonAbajo;
  estadoBotonAbajo = digitalRead(ABAJO);
  if (estadoBotonAbajo == LOW) {
    flagBotonAbajo = presionado;
    Serial.print("Boton ABAJO apretado: ");
    Serial.println(flagBotonAbajo);
  }
  estadoBotonAbajo = digitalRead(ABAJO);
  if (estadoBotonAbajo == HIGH && flagBotonAbajo == presionado) {
    variableBotonAbajo = variableBotonAbajo - cantidadResta;
    Serial.println("Boton ABAJO soltado");
    Serial.println(variableBotonAbajo);
    flagBotonAbajo = no_presionado;
  }
  return variableBotonAbajo;
}

int lecturaBotonArriba(int variableBotonArriba, int cantidadSuma) {
  int estadoBotonArriba;
  estadoBotonArriba = digitalRead(ARRIBA);
  if (estadoBotonArriba == LOW) {
    flagBotonArriba = presionado;
    Serial.print("Boton ARRIBA apretado: ");
    Serial.println(flagBotonArriba);
  }
  estadoBotonArriba = digitalRead(ARRIBA);
  if (estadoBotonArriba == HIGH && flagBotonArriba == presionado) {
    variableBotonArriba = variableBotonArriba + cantidadSuma;
    Serial.println("Boton ARRIBA soltado");
    Serial.println(variableBotonArriba);
    flagBotonArriba = no_presionado;
  }
  return variableBotonArriba;
}

void lecturaBotonesMenu() {
  estadoBotonIzquierda = digitalRead(IZQUIERDA);
  if (estadoBotonIzquierda == LOW) {
    flagBotonIzquierda = presionado;
    Serial.println("Boton IZQUIERDA apretado");
    Serial.println(flagBotonIzquierda);
  }
  estadoBotonIzquierda = digitalRead(IZQUIERDA);
  if (estadoBotonIzquierda == HIGH && flagBotonIzquierda == presionado) {
    contador--;
    if (contador <= -1) {
      contador = 0;
    }
    Serial.println("Boton IZQUIERDA soltado");
    Serial.print("Contador: ");
    Serial.println(contador);
    flagBotonIzquierda = no_presionado;
  }

  estadoBotonDerecha = digitalRead(DERECHA);
  if (estadoBotonDerecha == LOW) {
    flagBotonDerecha = presionado;
    Serial.println("Boton DERECHA apretado");
    Serial.println(flagBotonDerecha);
  }
  estadoBotonDerecha = digitalRead(DERECHA);
  if (estadoBotonDerecha == HIGH && flagBotonDerecha == presionado) {
    contador++;
    if (contador > 4) {
      contador = 4;
    }
    Serial.println("Boton DERECHA soltado");
    Serial.print("Contador: ");
    Serial.println(contador);
    flagBotonDerecha = no_presionado;
  }
}
