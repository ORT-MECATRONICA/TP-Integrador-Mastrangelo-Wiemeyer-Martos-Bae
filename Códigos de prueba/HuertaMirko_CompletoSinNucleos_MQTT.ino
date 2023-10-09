// Bae, Martos, Matrangelo, Wiemeyer

//Sensor temperatura
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

//Telegram
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
const char* ssid = "ORT-IoT";
const char* password = "OrtIOTnew22$2";
#define BOTtoken "6011340543:AAHn3L9_PeL9hfO5barxWzuc3tqZZbR3AhU"
#define CHAT_ID "-807022472"
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

//Display
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

//BOTONES
#define ABAJO 32
#define ARRIBA 27
#define DERECHA 25
#define IZQUIERDA 26
#define BOTON_OK 33     ////////// Pin 33 tiene al reves el pull up
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
int VALOR_UMBRAL_TEMP = 25;
int VALOR_UMBRAL_HUM;
int VALOR_GMT;
int VALOR_MQTT;

//Funcion alarma con temperatura, led, buzzer y cooler
#define LED_VERDE 13
#define LED_AMARILLO 12
#define LED_ROJO 14
#define BUZZER 17
#define COOLER 16
int lecturaTemperatura;

int maquinaEstadoAlarma;
#define VERDE 0
#define AMARILLO 1
#define ROJO 2
#define ROJO_SIN_BUZZER 3
boolean flagAlarma;

//MQTT
#include <AsyncMqttClient.h>
#include <time.h>
#include <Arduino.h>
const char name_device = 15;  //device numero de grupo 5A 1x siendo x el numero de grupo

unsigned long now = millis(); ///valor actual
unsigned long lastMeasure1 = 0; ///variable para contar el tiempo actual
unsigned long lastMeasure2 = 0; ///variable para contar el tiempo actual

const unsigned long interval_envio = VALOR_MQTT / 2; //Intervalo de envio de datos mqtt
const unsigned long interval_leo =  VALOR_MQTT;//Intervalo de lectura de datos y guardado en la cola
int i = 0;

long unsigned int timestamp ;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
const long  gmtOffset_sec = -10800;
const int   daylightOffset_sec = 0;

int indice_entra = 0; ///variables ingresar a la cola struct
int indice_saca = 0;
bool flag_vacio = 1;

#define MQTT_HOST IPAddress(10, 162, 24, 47)
#define MQTT_PORT 1883
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[150] ;  /////
// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

typedef struct
{
  long time;
  float T1;///tempe
  float H1;///humedad valor entre 0 y 100
  float luz;
  bool Alarma;
} estructura ;

const int valor_max_struct = 1000; ///valor vector de struct
estructura datos_struct [valor_max_struct];///Guardo valores hasta que lo pueda enviar
estructura aux2 ;

//Nucleos
/*TaskHandle_t tareaAlarma;
  TaskHandle_t tareaSensores;*/

void setup() {
  Serial.begin (9600);
  lcd.begin ();
  lcd.backlight();
  lcd.setCursor (0, 0);

  pinMode(ABAJO, INPUT_PULLUP);
  pinMode(ARRIBA, INPUT_PULLUP);
  pinMode(DERECHA, INPUT_PULLUP);
  pinMode(IZQUIERDA, INPUT_PULLUP);
  pinMode(BOTON_OK, INPUT_PULLUP);

  pinMode(PIN_HUMEDAD, INPUT);
  pinMode(PIN_LDR, INPUT);

  // Temperatura y telegram
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 Sensor event test"));

  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  //status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), 16);
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
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  // Bloquea el programa si no se puede conectar a internet
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  bot.sendMessage(CHAT_ID, "Bot Hola mundo", "");

  //xTaskCreatePinnedToCore(alarma, "alarma", 10000, NULL, 0, &tareaAlarma, 0);
  //xTaskCreatePinnedToCore(sensores, "sensores", 10000, NULL, 0, &tareaSensores, 1);

  setupmqtt(); //declaro pines digitales
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //Setup de time


}

void loop() {
  //mqtt
  now = millis();
  if (now - lastMeasure1 > interval_envio) { //envio el doble de lectura por si falla algun envio
    lastMeasure1 = now; // cargo el valor actual de millis
    fun_envio_mqtt(); //envio los valores por mqtt
  }
  if (now - lastMeasure2 > interval_leo) {
    lastMeasure2 = now; // cargo el valor actual de millis
    fun_entra(); //ingreso los valores a la cola struct
  }

  //sensores
  sensors_event_t temp_event;
  bmp_temp->getEvent(&temp_event);
  lecturaTemperatura = temp_event.temperature;

  estadoBotonOk = digitalRead(BOTON_OK);

  switch (maquinaEstadoAlarma) {
    case VERDE:
      flagAlarma = FALSE;
      digitalWrite(LED_VERDE, HIGH);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_ROJO, LOW);
      digitalWrite(BUZZER, LOW);
      digitalWrite(COOLER, LOW);
      if (lecturaTemperatura >= VALOR_UMBRAL_TEMP || lecturaHumedad <= VALOR_UMBRAL_HUM) {
        maquinaEstadoAlarma = ROJO;
      }
      break;

    case AMARILLO:
      flagAlarma = FALSE;
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_AMARILLO, HIGH);
      digitalWrite(LED_ROJO, LOW);
      digitalWrite(BUZZER, LOW);
      digitalWrite(COOLER, LOW);
      if (estadoBotonOk == presionado) {
        maquinaEstadoAlarma = VERDE;
      }
      if (lecturaTemperatura >= VALOR_UMBRAL_TEMP || lecturaHumedad <= VALOR_UMBRAL_HUM) {
        maquinaEstadoAlarma = ROJO;
      }
      break;

    case ROJO:
      flagAlarma = TRUE;
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_ROJO, HIGH);
      digitalWrite(BUZZER, HIGH);
      digitalWrite(COOLER, HIGH);
      Serial.println("La temperatura ha superado el umbral establecido");
      //bot.sendMessage(CHAT_ID, "La temperatura ha superado el umbral establecido", "");
      if (estadoBotonOk == presionado) {
        maquinaEstadoAlarma = ROJO_SIN_BUZZER;
      }
      if (lecturaTemperatura < VALOR_UMBRAL_TEMP || lecturaHumedad > VALOR_UMBRAL_HUM) {
        maquinaEstadoAlarma = AMARILLO;
      }
      break;

    case ROJO_SIN_BUZZER:
      flagAlarma = TRUE;
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_ROJO, HIGH);
      digitalWrite(BUZZER, LOW);
      digitalWrite(COOLER, LOW);
      if (lecturaTemperatura < VALOR_UMBRAL_TEMP || lecturaHumedad > VALOR_UMBRAL_HUM) {
        maquinaEstadoAlarma = AMARILLO;
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

    case MENU:                 //Pantalla 0: Para ver los valores leidos de temperatura, humedad y luz
      //Print del display
      //Serial.println("Modo: MENU");
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.setCursor(5, 0);
      char cadenaTemperatura[3];
      sprintf(cadenaTemperatura, "%03d", lecturaTemperatura);
      lcd.print (cadenaTemperatura);
      lcd.setCursor(9, 0);
      lcd.print("Hum:");
      lcd.setCursor(13, 0);
      lcd.print(lecturaHumedad);
      lcd.setCursor(0, 1);
      lcd.print("Luz:");
      lcd.setCursor(4, 1);
      char cadenaLDR[3];
      sprintf(cadenaLDR, "%03d", lecturaLDR);
      lcd.print (cadenaLDR);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu ();
      if (contador == 1) {
        Serial.println("Entrando a modo CAMBIO TEMP");
        lcd.clear();
        estado = CAMBIO_TEMP;
      }

      //Lo que tiene que suceder en este estado - Mostrar los valores leídos de temperatura, humedad y luz
      lecturaHumedad = map(analogRead(PIN_HUMEDAD), 0, 4096, 100, 0);
      lecturaLDR = map(analogRead(PIN_LDR), 0, 4096, 0, 100);

      break;

    case CAMBIO_TEMP:         //Pantalla 1: Para cambiar el valor umbral de temperatura
      //Print del display
      //Serial.print("Modo: CAMBIO_TEMP - ");
      lcd.setCursor(0, 0);
      lcd.print ("MODO CAMBIO_TEMP");
      lcd.setCursor(0, 1);
      lcd.print ("Valor: ");
      char cadena[5];
      sprintf(cadena, "%02d", VALOR_UMBRAL_TEMP);
      lcd.print (cadena);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu ();
      if (contador == 0) {
        lcd.clear();
        estado = MENU;
      }
      if (contador == 2) {
        lcd.clear();
        estado = CAMBIO_HUM;
      }

      //Lo que tiene que suceder en este estado
      VALOR_UMBRAL_TEMP = lecturaBotonAbajo (VALOR_UMBRAL_TEMP, uno);
      VALOR_UMBRAL_TEMP = lecturaBotonArriba (VALOR_UMBRAL_TEMP, uno);
      Serial.print("Valor TEMP: ");
      Serial.println(VALOR_UMBRAL_TEMP);

      //Serial.print(" - Contador: ");
      //Serial.println(contador);

      break;

    case CAMBIO_HUM:         //Pantalla 2: Para cambiar el valor umbral de hunedad
      //Print del display
      //Serial.print("Modo: CAMBIO_HUM - ");
      lcd.setCursor(0, 0);
      lcd.print ("MODO CAMBIO_HUM");
      lcd.setCursor(0, 1);
      lcd.print ("Valor: ");
      char cadena1[5];
      sprintf(cadena1, "%02d", VALOR_UMBRAL_HUM);
      lcd.print (cadena);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu ();
      if (contador == 1) {
        lcd.clear();
        estado = CAMBIO_TEMP;
      }
      if (contador == 3) {
        lcd.clear();
        estado = CAMBIO_GMT;
      }

      //Lo que tiene que suceder en este estado
      VALOR_UMBRAL_TEMP = lecturaBotonAbajo (VALOR_UMBRAL_HUM, uno);
      VALOR_UMBRAL_TEMP = lecturaBotonArriba (VALOR_UMBRAL_HUM, uno);
      Serial.print("Valor HUM: ");
      Serial.println(VALOR_UMBRAL_HUM);

      //Serial.print(" - Contador: ");
      //Serial.println(contador);

      break;

    case CAMBIO_GMT:       //Pantalla 3: Para cambiar el valor de gmt
      //Print del display
      Serial.print("Modo: CAMBIO_GMT - ");
      lcd.setCursor(0, 0);
      lcd.print ("MODO CAMBIO_GMT");
      lcd.setCursor(0, 1);
      lcd.print ("Hora: ");
      char cadena2[5];
      sprintf(cadena2, "%05d", VALOR_GMT);
      lcd.print (cadena2);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu ();
      if (contador == 2) {
        lcd.clear();
        estado = CAMBIO_TEMP;
      }
      if (contador == 4) {
        lcd.clear();
        estado = CAMBIO_MQTT;
      }

      //Lo que tiene que suceder en este estado
      VALOR_GMT = lecturaBotonAbajo (VALOR_GMT, tresMil);
      VALOR_GMT = lecturaBotonArriba (VALOR_GMT, tresMil);
      Serial.print("Hora GMT: ");
      Serial.print(VALOR_GMT);

      Serial.print(" - Contador: ");
      Serial.println(contador);

      break;

    case CAMBIO_MQTT:       //Pantalla 4: Para cambiar el tiempo de mqtt
      //Print del display
      Serial.print("Modo: CAMBIO_MQTT - ");
      lcd.setCursor(0, 0);
      lcd.print ("MODO CAMBIO_MQTT");
      lcd.setCursor(0, 1);
      lcd.print ("Valor: ");
      char cadena3[5];
      sprintf(cadena3, "%04d", VALOR_MQTT);
      lcd.print (cadena3);

      //Lectura de botones para moverse entre pantallas
      lecturaBotonesMenu ();
      if (contador == 2) {
        lcd.clear();
        estado = CAMBIO_GMT;
      }

      //Lo que tiene que suceder en este estado
      VALOR_MQTT = lecturaBotonAbajo (VALOR_MQTT, sesenta);
      VALOR_MQTT = lecturaBotonArriba (VALOR_MQTT, sesenta);
      if (VALOR_MQTT < 0) {
        VALOR_MQTT = 0;
      }
      Serial.print("Valor MQTT: ");
      Serial.print(VALOR_MQTT);

      Serial.print(" - Contador: ");
      Serial.println(contador);

      interval_envio = VALOR_MQTT / 2;
      interval_leo = VALOR_MQTT;

      break;
  }
}

int lecturaBotonAbajo (int variableBotonAbajo, int cantidadResta) {
  int estadoBotonAbajo;
  estadoBotonAbajo = digitalRead (ABAJO);
  if (estadoBotonAbajo == LOW) {
    flagBotonAbajo = presionado;
    Serial.print("Boton ABAJO apretado: ");
    Serial.println(flagBotonAbajo);
  }
  estadoBotonAbajo = digitalRead (ABAJO);
  if (estadoBotonAbajo == HIGH && flagBotonAbajo == presionado) {
    variableBotonAbajo = variableBotonAbajo - cantidadResta;
    Serial.println("Boton ABAJO soltado");
    Serial.println(variableBotonAbajo);
    flagBotonAbajo = no_presionado;
  }
  return variableBotonAbajo;
}

int lecturaBotonArriba (int variableBotonArriba, int cantidadSuma) {
  int estadoBotonArriba;
  estadoBotonArriba = digitalRead (ARRIBA);
  if (estadoBotonArriba == LOW) {
    flagBotonArriba = presionado;
    Serial.print("Boton ARRIBA apretado: ");
    Serial.println(flagBotonArriba);
  }
  estadoBotonArriba = digitalRead (ARRIBA);
  if (estadoBotonArriba == HIGH && flagBotonArriba == presionado) {
    variableBotonArriba = variableBotonArriba + cantidadSuma;
    Serial.println("Boton ARRIBA soltado");
    Serial.println(variableBotonArriba);
    flagBotonArriba = no_presionado;
  }
  return variableBotonArriba;
}

void lecturaBotonesMenu () {
  estadoBotonIzquierda = digitalRead (IZQUIERDA);
  if (estadoBotonIzquierda == LOW) {
    flagBotonIzquierda = presionado;
    Serial.println("Boton IZQUIERDA apretado");
    Serial.println(flagBotonIzquierda);
  }
  estadoBotonIzquierda = digitalRead (IZQUIERDA);
  if (estadoBotonIzquierda == HIGH && flagBotonIzquierda == presionado) {
    contador --;
    if (contador <= -1) {
      contador = 0;
    }
    Serial.println("Boton IZQUIERDA soltado");
    Serial.print("Contador: ");
    Serial.println(contador);
    flagBotonIzquierda = no_presionado;
  }

  estadoBotonDerecha = digitalRead (DERECHA);
  if (estadoBotonDerecha == LOW) {
    flagBotonDerecha = presionado;
    Serial.println("Boton DERECHA apretado");
    Serial.println(flagBotonDerecha);
  }
  estadoBotonDerecha = digitalRead (DERECHA);
  if (estadoBotonDerecha == HIGH && flagBotonDerecha == presionado) {
    contador ++;
    if (contador > 4) {
      contador = 4;
    }
    Serial.println("Boton DERECHA soltado");
    Serial.print("Contador: ");
    Serial.println(contador);
    flagBotonDerecha = no_presionado;
  }
}

//setup wifi
void setupmqtt()
{
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}

//Funcion que envia valores cuando la estructura no este vacia
void fun_envio_mqtt ()
{
  fun_saca ();////veo si hay valores nuevos
  if (flag_vacio == 0) ////si hay los envio
  {
    Serial.print("enviando");
    ////genero el string a enviar
    snprintf (mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%u", name_device, aux2.time, aux2.T1, aux2.H1, aux2.luz, aux2.Alarma); //random(10,50)
    aux2.time = 0; ///limpio valores
    aux2.T1 = 0;
    aux2.H1 = 0;
    aux2.luz = 0;
    aux2.Alarma = 0;
    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    // Publishes Temperature and Humidity values
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
  }
  else
  {
    Serial.println("no hay valores nuevos");
  }
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

//Funcion que saca un valor de la estructura para enviar
void fun_saca () {
  if (indice_saca != indice_entra)
  {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.H1 = datos_struct[indice_saca].H1;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.Alarma = datos_struct[indice_saca].Alarma;
    flag_vacio = 0;

    Serial.println(indice_saca);
    if (indice_saca >= (valor_max_struct - 1))
    {
      indice_saca = 0;
    }
    else
    {
      indice_saca++;
    }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  }
  else
  {
    flag_vacio = 1; ///// no hay datos
  }
  return ;
}

//funcion que ingresa valores a la cola struct
void fun_entra (void)
{
  if (indice_entra >= valor_max_struct)
  {
    indice_entra = 0; ///si llego al maximo de la cola se vuelve a cero
  }
  // timestamp consigo la hora
  Serial.print("> NTP Time:");
  timestamp =  time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // fin de consigo la hora
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].T1 = lecturaTemperatura; // leo los datos //aca va la funcion de cada sensor
  datos_struct[indice_entra].H1 = lecturaHumedad; // se puede pasar por un parametro valor entre 0 y 100
  datos_struct[indice_entra].luz = lecturaLDR;
  datos_struct[indice_entra].Alarma = flagAlarma;
  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}
