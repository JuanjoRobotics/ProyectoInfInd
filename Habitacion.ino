#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <Arduino_JSON.h>

// datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("192.168.200.84")       // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Path to update firmware
#define HTTP_OTA_PORT         1880                     // Port of update server
                                                       // Name of firmware
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu" 

ADC_MODE(ADC_VCC);
DHTesp dht;
// Update these with values suitable for your network.

//Estructura para de json para publicar los datos
struct registro_datos {
  float temperatura;
  float humedad;
  float vcc;
  unsigned long tiempo;
  long rssi;
  String ssid;
  String mqtt_server;
  float LED;
  
  };

  struct registro_conexion { //estructura para el estado de la conexion
  bool conexion;
  
  };

const char* ssid = "MOVISTAR_D342";
const char* password = "2i4iroQrPPtZq8TxfmD2"; //En serio, esta es mi contraseña
const char* mqtt_server = "iot.ac.uma.es";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];
int value = 0;
double LED_dato;

// OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);

//------------------------------------------------------------------------------------
//Esta funcion nos convierte a un String todos los datos
//para ser interpretados por Json
String serializa_JSON2 (struct registro_datos datos)
{
  JSONVar jsonRoot;
  JSONVar DHT11;
  JSONVar Wifi;
  JSONVar ANALOG;
  String jsonString;
  
  DHT11["temp"] = datos.temperatura;
  DHT11["hum"] = datos.humedad;
  Wifi["ssid"] = datos.ssid;
  Wifi["ip"] = datos.mqtt_server;
  Wifi["rssi"] = datos.rssi;
  
  jsonRoot["Uptime"]= datos.tiempo;
  jsonRoot["vcc"] = datos.vcc/1000.;
  jsonRoot["DHT11"]= DHT11;
  jsonRoot["Wifi"]=Wifi;
  jsonRoot["LED"]=datos.LED;
   
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
    estado.conexion=false;
    
    if (client.connect(clientId.c_str(),"infind","zancudo","infind/GRUPO2/conexion1/habitacion",0,true,serializa_JSON(estado).c_str())) {
      Serial.println("connected");
      
      estado.conexion = true;
      //"{\"online\":false}"
     client.publish("infind/GRUPO2/conexion/habitacion",serializa_JSON(estado).c_str(),true);
     
     client.subscribe("infind/GRUPO2/led/cmd/habitacion"); //Me suscribo al topic del estado del led
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//---------------------------------------------------------------------------------

void callback(char* topic, byte* payload, unsigned int length) {
  char* mensaje=(char*)malloc(length+1); // reservo memoria para copia del mensaje
  strncpy(mensaje,(char*)payload,length); // copio el mensaje en cadena de caracteres

  // compruebo que es el topic adecuado
  if(strcmp(topic,"infind/GRUPO2/led/cmd/habitacion")==0)
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
     char msg[128];
      
     double subirled=LED_dato;   
     while(subirled<valor)
      {
     double PWM = 1023*(1-subirled/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",subirled);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/habitacion",msg2 ); //Publico que he recibido el dato del led
     subirled++;
     delay(100);
    }
         while(subirled>valor)
      {
     double PWM = 1023*(1-subirled/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",subirled);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/habitacion",msg2 ); //Publico que he recibido el dato del led
     subirled--;
     delay(100);
    }
    
    double PWM = 1023*(1-valor/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",valor);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/habitacion",msg2 ); //Publico que he recibido el dato del led
     
    LED_dato = valor;
     
  free(mensaje); // libero memoria
}
  }
}
//------------------------------------------------------------------------------------------------------
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);
  client.setBufferSize(512); //Ampliamos el tamaño del buffer
  char cadena[512];
  dht.setup(5, DHTesp::DHT11); // Connect DHT sensor to GPIO 5

  // OTA
  Serial.println( "---------CAMBIA-----------------" );
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
}
//------------------------------------------------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------------------------------------------------

void loop() {
  
 String mensaje_conexion;
  if (!client.connected()) {
    reconnect();
  }
    
  client.loop();

  struct registro_datos misdatos; //me creo una estructura de cada tipo
  
  
  unsigned long now = millis();
  if (now - lastMsg > 10005) {
    
  
   //Las variables que vamos a meter en nuestra estructura datos
   misdatos.ssid= WiFi.localIP().toString();
   misdatos.mqtt_server= WiFi.SSID(); 
   misdatos.vcc = ESP.getVcc();
   misdatos.rssi = WiFi.RSSI();
   misdatos.temperatura = dht.getTemperature();
   misdatos.humedad = dht.getHumidity();
   misdatos.tiempo = millis();
   misdatos.LED = LED_dato;
//Publicamos los datos
  client.publish("infind/GRUPO2/datos/habitacion",serializa_JSON2(misdatos).c_str() );
//Publicamos el estado de conexion con retain flag=true

  
    lastMsg = now;
  }
}
