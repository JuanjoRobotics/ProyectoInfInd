#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino_JSON.h>

// LÁSER
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// CALIDAD AIRE
#include <MQ2Lib.h>
int pin = 0; //Pin al que se conectará de forma analógica
MQ2 mq2(pin, true); // Habilitamos el pin

// DHT
#include "DHTesp.h"
ADC_MODE(ADC_VCC);
DHTesp dht;

// Datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("192.168.0.241")       // Dirección del servidor de actualización OTA 192.168.1.113
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Campo firmware para actualizar
#define HTTP_OTA_PORT         1880                     // Port of update server
                                                       // Nombre del firmware
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu" 

#define OTA_URL "https://iot.ac.uma.es:1880/esp8266-ota/update"// Address of OTA update server
#define HTTP_OTA_VERSION   String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1)+".nodemcu"
String OTAfingerprint("5D 56 09 5C 5F 7B A4 3F 01 B7 22 31 D3 A7 DA A3 6E 10 2E 60"); // sustituir valor

// -----------------------ESTRUCTURAS JSON-----------------------

//Estructura de json para publicar los datos globales
struct registro_datos {
  float temperatura;
  float humedad;
  float vcc;
  unsigned long tiempo;
  long rssi;
  String ssid;
  String mqtt_server;
  float LED;
  float lpg;
  float co;
  float humo;
  char CHIPI;
  };
  
// Estructura de json para publicar conexión
  struct registro_conexion { //estructura para el estado de la conexion
  bool conexion;
  };
  
  // Estructura de json para publicar actualización
  struct registro_actualizacion { //estructura para el estado de la actualización
  bool actualizacion;
  };
  
// Estructura de json para publicar distancia LÁSER
    struct registro_distancia { //estructura para el estado de la conexion
  float distancia;
  };

// -----------------------INTERRUPCIONES-----------------------
//Pulsador - Pulsa UNA o DOS veces para encender o apagar el GPIO 2 o el GPIO 16

// Declaración Variables
int boton_flash=0;       // GPIO0 = boton flash
int estado_polling=HIGH; // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
volatile int estado_int=HIGH;     // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW

unsigned long inicio_pulso_polling = 0; // inicialización a 0 del pulso

volatile unsigned long ahora; // Se pondrá como global porque también se usará en el LOOP() - tiempo en que se da la actualización
volatile unsigned long penultima_int = 0; // Ayuda al envío de ultimo_dato antes de actualizarlo al valor actual
unsigned long ultima_int = 0; // Para la última interrupción

volatile bool interrupcion=false; // Se usará esta variable para indicar cuándo se da una interrupción

/*------------------  RTI  --------------------*/
// Rutina de Tratamiento de la Interrupcion (RTI)
ICACHE_RAM_ATTR void RTI() {
  
  int lectura=digitalRead(boton_flash); //Se toma el valor en alta o baja del botón flash
  interrupcion=true; // Inicialmente, salta la Interrupción por lo que se activa su variable correspondiente
  ahora= millis(); // se toma el valor actal en ms
  
  // Para eliminar rebotes:
  if(lectura==estado_int || ahora-ultima_int<50) return; // filtro rebotes 50ms
  if(lectura==LOW) // No está pulsado el botón flash
  { 
   estado_int=LOW; 
  }
  else // Está pulsado el botón flash
  {
   estado_int=HIGH; 
   penultima_int=ultima_int; // Se toma el valor del tiempo en que se dio la penúltima interrupción 
  }                          // para que se tenga constancia de cuándo se ha soltado el botón
  ultima_int = ahora; 
}

// -----------------------DECLARACION--DE--VARIABLES-----------------------
// Wifi - MQTT
const char* ssid = "MiFibra-D95E"; // JAVI: ALFIL_PISOS // LEO: vodafoneAAR6W7 // 
const char* password = "YPYQRZg7"; // JAVI: alfil2020 // LEO: Hn3fF6xXYFbgPJsH
const char* mqtt_server = "iot.ac.uma.es"; // Servidor MQTT común a todos los integrantes del grupo

WiFiClient espClient;

PubSubClient client(espClient);

// Se define el ancho del buffer para las estructuras json
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];
char msg3[MSG_BUFFER_SIZE];

int value = 0;
double LED_dato;

// OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);

// Interrupciones
unsigned long lastMsg = 0;
bool actualiza = false; 
bool cuenta_pulsos=true;
bool pulsaciones=false;

unsigned long inicio = 0; 
int cont=0;

int LED_Secundario = 16; //GPIO 16 - LED de ABAJO
//int BUILTIN_LED = 2; // GPIO 0 - LED de ARRIBA
int estado_led2=1; // Inicialmente, ambos LEDs están encendidos: estado=1. En caso de estar apagados, estado=0
int estado_led16=1;

//Recogida de datos
double TiempoRecogida=10000; // Recoger datos sensores
double TiempoLED=10; // Tiempo en que tarda en cambiar el LED un 1%

// LASER
unsigned long lastMsgLASER = 0;
float cambia_dist = 0;

//-----------------------CONVERSIÓN A STRING----ESTRUCTURAS JSON-----------------------
//Esta funcion nos convierte a un String todos los datos globales para ser interpretados por Json
String serializa_JSON2 (struct registro_datos datos)
{
  JSONVar jsonRoot;
  JSONVar DHT11;
  JSONVar Wifi;
  JSONVar MQ2;
  JSONVar ANALOG;
  String jsonString;
  
  DHT11["temp"] = datos.temperatura;
  DHT11["hum"] = datos.humedad;
  Wifi["ssid"] = datos.ssid;
  Wifi["ip"] = datos.mqtt_server;
  Wifi["rssi"] = datos.rssi;
  MQ2["LPG"] = datos.lpg; // Gas natural
  MQ2["CO"] = datos.co; 
  MQ2["HUMO"] = datos.humo;

  jsonRoot["CHIPID"]=datos.CHIPI; // ID del Chip
  jsonRoot["Uptime"]= datos.tiempo;
  jsonRoot["vcc"] = datos.vcc/1000.;
  jsonRoot["DHT11"]= DHT11;
  jsonRoot["Wifi"]=Wifi;
  jsonRoot["LED"]=datos.LED;
  jsonRoot["MQ-2"]=MQ2;

  return JSON.stringify(jsonRoot);
}
// -----------------------
// Función para la conexion
String serializa_JSON (struct registro_conexion estado)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["online"] = estado.conexion;
  
  return JSON.stringify(jsonRoot);
}
// -----------------------
// Función para la actualización
String serializa_JSONact (struct registro_actualizacion estado_act)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["actualiza:"] = estado_act.actualizacion;
  
  return JSON.stringify(jsonRoot);
}
// -----------------------

// Función para la distancia LÁSER
String serializa_JSONdistist (struct registro_distancia laser)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["VL53L0X"] = laser.distancia;
  
  return JSON.stringify(jsonRoot);
}
  
//-----------------------CONEXIÓN A INTERNET-----------------------
//setup de la conexion a internet
void setup_wifi() {

  delay(10);
  // Empezamos conectándonos a la red WiFi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) { // Una vez conectados, sale del bucle while
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  Serial.printf("Connected, mac address: %02X:%02X:%02X:%02X:%02X:%02X\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]); // MAC del dispositivo

}
//-----------------------CONEXIÓN MQTT-----------------------

//Bucle de conexion MQTT
void reconnect() {
  // Permanecemos en el bucle hasta que nos reconectemos
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Se crea un cliente ID aleatorio
    String clientId = "ESP8266Client-";
    clientId += String(ESP.getChipId()) ;
    // Esperamos a conexión
    struct registro_conexion estado; // Declaramos la estructura para conexión (json)
    struct registro_actualizacion estado_act; // Declaramos la estructura para actualización (json)
    estado.conexion=false; // Inicialmente, no hemos establecido conexión
    estado_act.actualizacion=false; // No se ha actualizado aún
    
    if (client.connect(clientId.c_str(),"infind","zancudo","infind/GRUPO2/ESP47/conexion/salon",0,true,serializa_JSON(estado).c_str())) {
      Serial.println("connected"); // Estaremos conectados. Usuario: infind. Contraseña: zancudo. 
      
      estado.conexion = true;
      //"{\"online\":false}"
     client.publish("infind/GRUPO2/ESP47/conexion/salon",serializa_JSON(estado).c_str(),true); // Publicamos por el topic correspondiente el estado de conexión
     
     client.subscribe("infind/GRUPO2/ESP47/led/cmd/salon"); //Me suscribo al topic del estado del led
     client.subscribe("infind/GRUPO2/ESP47/config/cmd/salon"); //Me suscribo al topic del estado del led
    } else { // En caso de no haber establecido conexión, reintenta a los 5 segundos
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//-----------------------CALLBACK-----------------------
void callback(char* topic, byte* payload, unsigned int length) { 
  char* mensaje=(char*)malloc(length+1); // reservo memoria para copia del mensaje
  strncpy(mensaje,(char*)payload,length); // copio el mensaje en cadena de caracteres

  // compruebo que es el topic adecuado
  if(strcmp(topic,"infind/GRUPO2/ESP47/led/cmd/salon")==0)
  {
 
    StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje
    // Deserializa el documento JSON
    DeserializationError error = deserializeJson(root, mensaje);

    // Compruebo si hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else // En caso de no haber error, se hace la secuencia propuesta:
    {
     double valor = root["level"]; // Se toma el valor del dato de entrada
           
     double subirled=LED_dato; // Iniciamos variable
     while(subirled<valor) // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
      {
     double PWM = 1023*(1-subirled/100);  // Se actualiza la intensidad del LED a través de PWM
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",subirled);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/ESP47/led/status/salon",msg2 ); //Publico que he recibido el dato del led
     subirled++;
     delay(TiempoLED); // Se espera el tiempo exigido por el usuario
    }
         while(subirled>valor) // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
      {
     double PWM = 1023*(1-subirled/100); 
     analogWrite(BUILTIN_LED,PWM); 
     
    sprintf(msg2," {\"led\": %f} ",subirled);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/ESP47/led/status/salon",msg2 ); 
     subirled--;
     delay(TiempoLED);
    }
    
    double PWM = 1023*(1-valor/100); //Finalmente, se alcanza el valor de LED propuesto por el usuario
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",valor);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/ESP47/led/status/salon",msg2 ); //Publico que he recibido el dato del led
     
    LED_dato = valor;
    if (valor==0) // Estudio del estado del LED (apagado o encendido)
    {
      Serial.println("GPIO 2 APAGADO");
      estado_led2=0;
    }
    else
    {
      Serial.println("GPIO 2 ENCENDIDO");
      estado_led2=1;
    }
  free(mensaje); // libero memoria
}
  }


    if(strcmp(topic,"infind/GRUPO2/ESP47/config/cmd/salon")==0) // Se estudia el estado del topic específico
    {
      StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(root, mensaje);

      // Compruebo si no hubo error
      if (error) {
        Serial.print("Error deserializeJson() failed: ");
        Serial.println(error.c_str());
      }
      else // En caso de no haber error:
      {
        double valorDatos = root["TDATOS"]; // Se toman los valores corresponientes al tiempo de recogida de datos y actualización del LED
        double valorLED = root["TLED"];
        //char msg[128];
        TiempoRecogida=valorDatos;
        TiempoLED=valorLED;
        sprintf(msg3,"{\"Tiempo de Recogida de Datos\": %f, \"Tiempo para subir el LED\": %f}",valorDatos, valorLED);
        //publica que he llegado a ese valor
        client.publish("infind/GRUPO2/ESP47/config/salon",msg3 ); //Publico que he recibido el dato del led

        free(mensaje); // libero memoria
      }
  }
}

//-----------------------SET-UP-----------------------
void setup() {
  struct registro_actualizacion estado_act; // Declaramos la estructura para actualización (json)
  pinMode(BUILTIN_LED, OUTPUT);    // Inicializamos el pin BUILTIN_LED como salida
  pinMode(LED_Secundario, OUTPUT); // Inicializamos el pin LED_Secundario como salida
  Serial.begin(9600);
  setup_wifi();
  // OTA -> Actualizamos al empezar
      client.subscribe("infind/GRUPO2/ESP47/FOTA/salon"); //Me suscribo al topic de la actualización
      Serial.println( "----------hhhhhhhhhhhhhhhhhhhhh----------------" );
      Serial.println( "Comprobando actualización:" );
      Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH); // Se comprueba la actualización
      Serial.println( "--------------------------" );  
      ESPhttpUpdate.setLedPin(16,LOW);
      ESPhttpUpdate.onStart(inicio_OTA);
      ESPhttpUpdate.onError(error_OTA);
      ESPhttpUpdate.onProgress(progreso_OTA);
      ESPhttpUpdate.onEnd(final_OTA);
      switch(ESPhttpUpdate.update(HTTP_OTA_ADDRESS, HTTP_OTA_PORT, HTTP_OTA_PATH, HTTP_OTA_VERSION)) {
        case HTTP_UPDATE_FAILED:
          Serial.printf(" HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println(F(" El dispositivo ya está actualizado"));
          break;
        case HTTP_UPDATE_OK:
          Serial.println(F(" OK"));
          break;
        } 
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);
 // client.setCallback(callbackRECOGIDA);
  client.setBufferSize(512); //Ampliamos el tamaño del buffer
  char cadena[512];
  dht.setup(5, DHTesp::DHT11); // Connect DHT sensor to GPIO 5

  //FLASH
   pinMode(boton_flash, INPUT_PULLUP); // Inicializamos el pin LED_Secundario como entrada
  // Activamos la interrupción
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);
  
  // LÁSER: Iniciar sensor
  Serial.println("VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Error al iniciar VL53L0X"));
    while(1); // Esperamos a que se inicie el sensor de distancia
  }
  
    //MQ-2
    mq2.begin();
}
//-----------------------OTA-----------------------
// OTA
void final_OTA()
{
  Serial.println("Fin OTA. Reiniciando...");
}

void inicio_OTA()
{
  Serial.println("Nuevo Firmware encontrado. Actualizando...");
}

void error_OTA(int e)
{
  char cadena[64];
  snprintf(cadena,64,"ERROR: %d",e);
  Serial.println(cadena);
}

void progreso_OTA(int x, int todo)
{
  char cadena[256];
  int progress=(int)((x*100)/todo);
  if(progress%10==0)
  {
    snprintf(cadena,256,"Progreso: %d%% - %dK de %dK",progress,x/1024,todo/1024);
    Serial.println(cadena);
  }
}
//-----------------------LOOP-----------------------
void loop() {
  // INTERRUPCIONES
  if (interrupcion==true) // En caso de que se active la interrupción
  {
    if (cuenta_pulsos=true) 
    {
      inicio=ahora; // Estudiamos el tiempo en que se produce la interrupción
      cuenta_pulsos=false;
    }
    if (estado_int==LOW) // Si el botón está pulsado, se toma el registro del tiempo en que se produce
    {
      Serial.print("Se detectó una pulsación en millis = ");
      Serial.print(ahora);
      Serial.println(" ms");
    }
    if (estado_int==HIGH) // Cuando se suelta el botón, se tomar el tiempo y se hace la diferencia, conociendo el tiempo que se mantiene pulsado
    {
      Serial.print("Terminó la pulsación, duración = ");
      Serial.print(ahora-penultima_int);
      Serial.println(" ms");
      cont++; // Se ha dado un pulso 
      if (ahora-penultima_int < 2000) // Si NO haya actualización, se estudian las pulsaciones
      {
        pulsaciones=true;
      }
      else // Si se mantiene +2seg pulsado, se habilita la señal que permite actualizar
        actualiza=true;
    }
    interrupcion=false; // Desabilitamos la interrupción hasta que vuelva a haber una llamada a la misma
    
    }
    if (actualiza==true) // mantenemos pulsado: ACTUALIZAMOS! 
    {
      client.subscribe("infind/GRUPO2/ESP47/FOTA/salon"); //Me suscribo al topic de la actualización cuando se mantenga más de 2 segundos el botón flash pulsado
      // OTA
      Serial.println( "--------------------------" );
      Serial.println( "Comprobando actualización:" );
      Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH); // Se comprueba la actualización
      Serial.println( "--------------------------" );  
      ESPhttpUpdate.setLedPin(16,LOW);
      ESPhttpUpdate.onStart(inicio_OTA);
      ESPhttpUpdate.onError(error_OTA);
      ESPhttpUpdate.onProgress(progreso_OTA);
      ESPhttpUpdate.onEnd(final_OTA);
      switch(ESPhttpUpdate.update(HTTP_OTA_ADDRESS, HTTP_OTA_PORT, HTTP_OTA_PATH, HTTP_OTA_VERSION)) {
        case HTTP_UPDATE_FAILED:
          Serial.printf(" HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println(F(" El dispositivo ya está actualizado"));
          break;
        case HTTP_UPDATE_OK:
          Serial.println(F(" OK"));
          break;
        } 
        actualiza=false;
    }
    unsigned long now=millis();
   
      if (now-inicio>650) // Cuando pasen 0.65 seg tras la interrupción
      {
        if (cont==2) // Si se han contado 2 pulsos -> encendemos o apagamos el LED principal (GPIO2)
        {
        Serial.println("DOBLE PULSACION");
        if (estado_led2==1) 
          {
            digitalWrite(BUILTIN_LED, HIGH);
            Serial.println("GPIO 2 APAGADO");
            estado_led2=0;
          }
          else
          { 
            digitalWrite(BUILTIN_LED, LOW);
            Serial.println("GPIO 2 ENCENDIDO");
            estado_led2=1;
          }
          cont=0; // Reseteamos la cuenta
          cuenta_pulsos=true;
        }
        if (cont==1) // Si solo se ha contado 1 pulsación, encendemos o apagamos el LED secundario (GPIO 16)
        {
        Serial.println("UNA PULSACION");
        if (estado_led16==1)
          {
            digitalWrite(LED_Secundario, HIGH);
            Serial.println("GPIO 16 APAGADO");
            estado_led16=0;
          }
          else
          { 
            digitalWrite(LED_Secundario, LOW);
            Serial.println("GPIO 16 ENCENDIDO");
            estado_led16=1;
          }
          cont=0; // Reseteamos la cuenta
          cuenta_pulsos=true;
        }
        else if (cont>2) // Si se cuentan más de 2 pulsos, se hace saber que no es válido
        {
          Serial.println("MAS DE 2 PULSACIONES, CÓDIGO NO VÁLIDO");

          cont=0;
          cuenta_pulsos=true;
        }
    delay(10);
    }
  // LÁSER
  struct registro_distancia laser; 
  VL53L0X_RangingMeasurementData_t measure;
  
  //Serial.print("Leyendo sensor... ");
  lox.rangingTest(&measure, false); // si se pasa true como parametro, muestra por puerto serie datos de debug
  float medida_laser = measure.RangeMilliMeter;
  laser.distancia = medida_laser;
  // CONEXIÓN
  String mensaje_conexion;
  if (!client.connected()) {
    reconnect();
  }
    
  client.loop();
  //unsigned long now = millis();

  // LASER
  if (cambia_dist >= medida_laser+20 || cambia_dist<=medida_laser-20) // cuando salgamos de un rango específico, se toma una nueva medida y se hace saber al usuario
  /*if (now - lastMsgLASER > 3000) {*/
   { 
   Serial.print("Distancia (mm): ");
   Serial.println(measure.RangeMilliMeter);
   client.publish("infind/GRUPO2/ESP47/datos/salon/puerta",serializa_JSONdistist(laser).c_str() ); // publicamos la distancia por el topic correspondiente
  
   cambia_dist=medida_laser; // Actualizamos la distancia 
  }

  // DATOS
  struct registro_datos misdatos; //me creo una estructura de cada tipo
  
  
  if (now - lastMsg > 10005) {
   //Las variables que vamos a meter en nuestra estructura datos
   // WiFi y MQTT
   misdatos.ssid= WiFi.localIP().toString();
   misdatos.mqtt_server= WiFi.SSID(); 
   misdatos.rssi = WiFi.RSSI();
   // DHT
   misdatos.temperatura = round(dht.getTemperature());
   misdatos.humedad = dht.getHumidity();
   // VCC
   misdatos.vcc = ESP.getVcc();
   // Tiempo
   misdatos.tiempo = millis();
   // LED
   misdatos.LED = LED_dato;
   // Calidad aire
   misdatos.lpg =mq2.readLPG();
   misdatos.co= mq2.readCO();
   misdatos.humo = mq2.readSmoke();
   // ID del CHIP
   misdatos.CHIPI = ESP.getChipId();
   
// Publicamos los datos por el topic correspondiente
  client.publish("infind/GRUPO2/ESP47/datos/salon",serializa_JSON2(misdatos).c_str() );
//Publicamos el estado de conexion con retain flag=true

    lastMsg = now; // Actualizamos el tiempo en que se produce la publicación de datos
  }
  delay(10);
}
