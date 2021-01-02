#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <Arduino_JSON.h>

// CALIDAD AIRE
#include <MQ2Lib.h>

// LECTURA DIGITAL CALIDAD AIRE
int pin = 0; //change this to the pin that you use
MQ2 mq2(pin, true);

// LÁSER
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


// datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("192.168.1.113")       // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Path to update firmware
#define HTTP_OTA_PORT         1880                     // Port of update server
                                                       // Name of firmware
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu" 

ADC_MODE(ADC_VCC);
DHTesp dht;
// Update these with values suitable for your network.

//Estructura de json para publicar los datos
struct registro_datos {
  float temperatura;
  float humedad;
  float vcc;
  unsigned long tiempo;
  long rssi;
  String ssid;
  String mqtt_server;
  float LED;
  int lpg;
  int co;
  int humo;
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

// -----------------------INTERRUPCIONES--------------------------------------------

//Pulsador - Pulsa 1 vez y se enciende o se apaga
int boton_flash=0;       // GPIO0 = boton flash
int estado_polling=HIGH; // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
volatile int estado_int=HIGH;     // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW

unsigned long inicio_pulso_polling = 0;

volatile unsigned long ahora; // SE PONDRÁ GLOBAL PARA EN LOOP() CONOCER EL VALOR ACTUAL QUE SE TOMA EN LA INTERRUPCIÓN
volatile unsigned long penultima_int = 0; // CREO ESTA VARIABLE PARA ENVIAR EL ULTIMO_DATO Y USARLO EN EL INT ANTES DE ACUTALIZARSE AL DATO ACTUAL
unsigned long ultima_int = 0;

volatile bool interrupcion=false; // SE USARÁ ESTA VARIABLE PARA INDICAR CUÁNDO SE DA UNA INTERRUPCIÓN

//int ledsecundario=0;
/*------------------  RTI  --------------------*/
// Rutina de Tratamiento de la Interrupcion (RTI)
ICACHE_RAM_ATTR void RTI() {
  
  int lectura=digitalRead(boton_flash);
  interrupcion=true;
  ahora= millis();
  
  // descomentar para eliminar rebotes
  if(lectura==estado_int || ahora-ultima_int<50) return; // filtro rebotes 50ms
  if(lectura==LOW)
  { 
   estado_int=LOW;
  }
  else
  {
   estado_int=HIGH;
   penultima_int=ultima_int;
  }
  ultima_int = ahora;
}

// -----------------------DECLARACION--DE--VARIABLES--------------------------------------------
const char* ssid = "MiFibra-D95E"; // JAVI: ALFIL_PISOS // LEO: vodafoneAAR6W7 // 
const char* password = "YPYQRZg7"; // JAVI: alfil2020 // LEO: Hn3fF6xXYFbgPJsH
const char* mqtt_server = "iot.ac.uma.es";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
unsigned long lastMsgLASER = 0;

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
bool actualiza = false;
bool cuenta_pulsos=true;
bool pulsaciones=false;

unsigned long inicio = 0; 
int cont=0;

int LED_Secundario = 16; //GPIO 16 - LED de ABAJO
//int BUILTIN_LED = 2; // GPIO 0 - LED de ARRIBA
int estado_led2=1;
int estado_led16=1;

bool cambia0=false;
bool cambia16=false;

//Recogida de datos
double TiempoRecogida=10000;
double TiempoLED=10;
// LASER
float cambia_dist = 0;

//------------------------------------------------------------------------------------
//Esta funcion nos convierte a un String todos los datos
//para ser interpretados por Json
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
  
  jsonRoot["Uptime"]= datos.tiempo;
  jsonRoot["vcc"] = datos.vcc/1000.;
  jsonRoot["DHT11"]= DHT11;
  jsonRoot["Wifi"]=Wifi;
  jsonRoot["LED"]=datos.LED;
  jsonRoot["MQ-2"]=MQ2;

  return JSON.stringify(jsonRoot);
}
//--------------------------------------------------------------------------------------
//Funcion para la conexion
String serializa_JSON (struct registro_conexion estado)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["online"] = estado.conexion;
  
  return JSON.stringify(jsonRoot);
}
//------------------------------------------------------------------------------------
//Funcion para la actualización
String serializa_JSONact (struct registro_actualizacion estado_act)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["actualiza:"] = estado_act.actualizacion;
  
  return JSON.stringify(jsonRoot);
}
//------------------------------------------------------------------------------------

// Función para la distancia LÁSER
String serializa_JSONdistist (struct registro_distancia laser)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["VL53L0X"] = laser.distancia;
  
  return JSON.stringify(jsonRoot);
}
  
//------------------------------------------------------------------------------------
//setup de la conexion a internet
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
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
  Serial.printf("Connected, mac address: %02X:%02X:%02X:%02X:%02X:%02X\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

}
//--------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------

//Bucle de conexion MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(ESP.getChipId()) ;
    // Attempt to connect
    struct registro_conexion estado;
    struct registro_actualizacion estado_act;
    estado.conexion=false;
    estado_act.actualizacion=false;
    
    if (client.connect(clientId.c_str(),"infind","zancudo","infind/GRUPO2/conexion/salon",0,true,serializa_JSON(estado).c_str())) {
      Serial.println("connected");
      
      estado.conexion = true;
      //"{\"online\":false}"
     client.publish("infind/GRUPO2/conexion/salon",serializa_JSON(estado).c_str(),true);
     
     client.subscribe("infind/GRUPO2/led/cmd/salon"); //Me suscribo al topic del estado del led
     client.subscribe("infind/GRUPO2/config/cmd/salon"); //Me suscribo al topic del estado del led
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//-------------------------------CALLBACK--------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  char* mensaje=(char*)malloc(length+1); // reservo memoria para copia del mensaje
  strncpy(mensaje,(char*)payload,length); // copio el mensaje en cadena de caracteres

  // compruebo que es el topic adecuado
  if(strcmp(topic,"infind/GRUPO2/led/cmd/salon")==0)
  {
 
    StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else
    {
     double valor = root["level"];
     //char msg[128];
      
     double subirled=LED_dato;   
     while(subirled<valor)
      {
     double PWM = 1023*(1-subirled/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",subirled);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/salon",msg2 ); //Publico que he recibido el dato del led
     subirled++;
     delay(TiempoLED);
    }
         while(subirled>valor)
      {
     double PWM = 1023*(1-subirled/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",subirled);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/salon",msg2 ); //Publico que he recibido el dato del led
     subirled--;
     delay(TiempoLED);
    }
    
    double PWM = 1023*(1-valor/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",valor);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/salon",msg2 ); //Publico que he recibido el dato del led
     
    LED_dato = valor;
    if (valor==0)
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


    if(strcmp(topic,"infind/GRUPO2/config/cmd/salon")==0)
  {
 
    StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else
    {
     double valorDatos = root["TDATOS"];
     double valorLED = root["TLED"];
     //char msg[128];
      TiempoRecogida=valorDatos;
      TiempoLED=valorLED;
    sprintf(msg3,"{\"Tiempo de Recogida de Datos\": %f, \"Tiempo para subir el LED\": %f}",valorDatos, valorLED);
     //publica que he llegado a ese valor
    client.publish("infind/GRUPO2/config/salon",msg3 ); //Publico que he recibido el dato del led

     
  free(mensaje); // libero memoria
}
  }

  
}

//------------------------------------SET-UP----------------------------------------------------------------
void setup() {
  struct registro_actualizacion estado_act;
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(LED_Secundario, OUTPUT); 
  Serial.begin(9600);
  setup_wifi();
  // OTA
      client.subscribe("infind/GRUPO2/FOTA/salon"); //Me suscribo al topic de la actualización
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
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);
 // client.setCallback(callbackRECOGIDA);
  client.setBufferSize(512); //Ampliamos el tamaño del buffer
  char cadena[512];
  dht.setup(5, DHTesp::DHT11); // Connect DHT sensor to GPIO 5

  //FLASH
   pinMode(boton_flash, INPUT_PULLUP);
  // descomentar para activar interrupción
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);
  
  // LÁSER: Iniciar sensor
  Serial.println("VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Error al iniciar VL53L0X"));
    while(1);
  }
  
    //MQ-2
    mq2.begin();
}
//----------------------------------------OTA--------------------------------------------------------------------------------
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
//-------------------------------------LOOP-----------------------------------------------------------------------------------
void loop() {
  // INTERRUPCIONES
  if (interrupcion==true)
  {
    if (cuenta_pulsos=true)
    {
      inicio=ahora;
      cuenta_pulsos=false;
    }
    if (estado_int==LOW)
    {
      Serial.print("Se detectó una pulsación en millis = ");
      Serial.print(ahora);
      Serial.println(" ms");
    }
    if (estado_int==HIGH)
    {
      Serial.print("Terminó la pulsación, duración = ");
      Serial.print(ahora-penultima_int);
      Serial.println(" ms");
      cont++;
      if (ahora-penultima_int < 2000) // Mientras NO haya actualización
      {
        pulsaciones=true;
      }
      else
        actualiza=true;
    }
    interrupcion=false; // DESABILITAMOS LA INTERRUPCIÓN HASTA QUE VUELVA A HABER UNA LLAMADA A LA MISMA
    
    }
    if (actualiza==true) // mantenemos pulsado: ACTUALIZAMOS! 
    {
      client.subscribe("infind/GRUPO2/FOTA/salon"); //Me suscribo al topic de la actualización
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
   
      if (now-inicio>650)
      {
        if (cont==2) // 2 pulsos - encendemos arriba
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
          cont=0;
          cuenta_pulsos=true;
        }
        if (cont==1)
        {
        Serial.println("UNA PULSACION");
        if (estado_led16==1)
          {
            digitalWrite(LED_Secundario, HIGH);
            Serial.println("GPIO 16 APAGADO");
            Serial.printf(" ESP8266 Chip id = %08X\n", ESP.getChipId());

            estado_led16=0;
          }
          else
          { 
            digitalWrite(LED_Secundario, LOW);
            Serial.println("GPIO 16 ENCENDIDO");
            estado_led16=1;
          }
          cont=0;
          cuenta_pulsos=true;
        }
        else if (cont>2)
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
  if (cambia_dist >= medida_laser+20 || cambia_dist<=medida_laser-20)
  /*if (now - lastMsgLASER > 3000) {*/
   { 
   Serial.print("Distancia (mm): ");
   Serial.println(measure.RangeMilliMeter);
   client.publish("infind/GRUPO2/datos/salon/puerta",serializa_JSONdistist(laser).c_str() );
   /*delay(3000);
   lastMsgLASER = now;*/
   cambia_dist=medida_laser;
  }

  // CALIDAD AIRE - MQ2
  /*read the values from the sensor, it returns
  *an array which contains 3 values.
  * 1 = LPG in ppm
  * 2 = CO in ppm
  * 3 = SMOKE in ppm
  */  
  // DATOS
  struct registro_datos misdatos; //me creo una estructura de cada tipo
  
  
  if (now - lastMsg > 10005) {
   //Las variables que vamos a meter en nuestra estructura datos
   misdatos.ssid= WiFi.localIP().toString();
   misdatos.mqtt_server= WiFi.SSID(); 
   misdatos.vcc = ESP.getVcc();
   misdatos.rssi = WiFi.RSSI();
   misdatos.temperatura = round(dht.getTemperature());
   misdatos.humedad = dht.getHumidity();
   misdatos.tiempo = millis();
   misdatos.LED = LED_dato;
   // Calidad aire
   misdatos.lpg =mq2.readLPG();
   misdatos.co= mq2.readCO();
   misdatos.humo = mq2.readSmoke();
   
//Publicamos los datos
  client.publish("infind/GRUPO2/datos/salon",serializa_JSON2(misdatos).c_str() );
//Publicamos el estado de conexion con retain flag=true

  
    lastMsg = now;
  }
  delay(10);
}
