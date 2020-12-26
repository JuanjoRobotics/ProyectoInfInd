
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <Arduino_JSON.h>
#include <MQ2Lib.h>


//MQ-2

int pin = 0; //change this to the pin that you use
MQ2 mq2(pin, true);

//CO <1.200ppm



// datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("192.168.1.90")       // Address of OTA update server
//#define HTTP_OTA_ADDRESS      F("192.168.200.84")       // Address of OTA update server
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
  float lpg;
  float co;
  float humo;
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

const char* ssid = "MiFibra-233E";
const char* password = "Lh7M2NMK"; //En serio, esta es mi contraseña

//const char* ssid = "ALFIL_PISOS";
//const char* password = "alfil2020"; //En serio, esta es mi contraseña

const char* mqtt_server = "iot.ac.uma.es";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];
char msg3[MSG_BUFFER_SIZE];
int value = 0;
double LED_dato;

//Pulsador
int boton_flash=0;       // GPIO0 = boton flash
int estado_polling=HIGH; // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
volatile int estado_int=HIGH;     // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
volatile bool interrupcion=false;
volatile unsigned long ahora;
unsigned long inicio_pulso_polling = 0;
volatile unsigned long penultima_int = 0;
unsigned long ultima_int = 0;
int ledboton=1;
int ledboton2=0;
double subirledboton=0;
int doblepulsacion=0;
int LED_Secundario=16; //Puerto GIOP 16
unsigned long lastInt = 0;


//Recogida de datos

double TiempoRecogida=10000;

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
  //Serial.print("Int en: ");
  //Serial.println(ahora);
  }
  else
  {
   estado_int=HIGH;
  // Serial.print("Int dura: ");
  // Serial.println(ahora-ultima_int);
  penultima_int=ultima_int;
  }
  ultima_int = ahora;
}


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
  JSONVar MQ2;
  JSONVar Wifi;
  JSONVar ANALOG;
  String jsonString;
  
  DHT11["temp"] = datos.temperatura*10/10.0;
  DHT11["hum"] = datos.humedad;
  MQ2["LPG"] = datos.lpg;
  MQ2["CO"] = datos.co;
  MQ2["Humo"] = datos.humo;
  Wifi["ssid"] = datos.ssid;
  Wifi["ip"] = datos.mqtt_server;
  Wifi["rssi"] = datos.rssi;
  
  jsonRoot["Uptime"]= datos.tiempo;
  jsonRoot["vcc"] = datos.vcc/1000.;
  jsonRoot["DHT11"]= DHT11;
  jsonRoot["MQ-2"]= MQ2;
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
    
    if (client.connect(clientId.c_str(),"infind","zancudo","infind/GRUPO2/conexion1/cocina",0,true,serializa_JSON(estado).c_str())) {
      Serial.println("connected");
      
      estado.conexion = true;
      //"{\"online\":false}"
     client.publish("infind/GRUPO2/conexion/cocina",serializa_JSON(estado).c_str(),true);
     
     client.subscribe("infind/GRUPO2/led/cmd/cocina"); //Me suscribo al topic del estado del led
     client.subscribe("infind/GRUPO2/RECOGIDA/cocina"); //Me suscribo al topic del tiempo de recogida de datos
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
  if(strcmp(topic,"infind/GRUPO2/led/cmd/cocina")==0)
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
     client.publish("infind/GRUPO2/led/status/cocina",msg2 ); //Publico que he recibido el dato del led
     subirled++;
     delay(10);
    }
         while(subirled>valor)
      {
     double PWM = 1023*(1-subirled/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",subirled);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/cocina",msg2 ); //Publico que he recibido el dato del led
     subirled--;
     delay(10);
    }
    
    double PWM = 1023*(1-valor/100);
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
    sprintf(msg2," {\"led\": %f} ",valor);
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/led/status/cocina",msg2 ); //Publico que he recibido el dato del led
     
    LED_dato = valor;
     
  free(mensaje); // libero memoria
}
  }


    if(strcmp(topic,"infind/GRUPO2/RECOGIDA/cocina")==0)
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
     double valor = root["nivel"];
     //char msg[128];
      TiempoRecogida=valor;
   sprintf(msg3," {\"Tiempo de recogida de datos\": %f} ",valor);
     //publica que he llegado a ese valor
    client.publish("infind/GRUPO2/TIEMPO/RECOGIDA/cocina",msg3 ); //Publico que he recibido el dato del led

     
  free(mensaje); // libero memoria
}
  }

  
}



//------------------------------------------------------------------------------------------------------
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(LED_Secundario, OUTPUT); 
  Serial.begin(9600);
  setup_wifi();
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

  //MQ-2
  mq2.begin();
  
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
  if (now - lastMsg > TiempoRecogida) {

  
   //Las variables que vamos a meter en nuestra estructura datos
   misdatos.ssid= WiFi.localIP().toString();
   misdatos.mqtt_server= WiFi.SSID(); 
   misdatos.vcc = ESP.getVcc();
   misdatos.rssi = WiFi.RSSI();
   misdatos.temperatura = dht.getTemperature();
   misdatos.humedad = dht.getHumidity();
   misdatos.lpg =mq2.readLPG();
   misdatos.co= mq2.readCO();
   misdatos.humo = mq2.readSmoke();
   misdatos.tiempo = millis();
   misdatos.LED = LED_dato;
//Publicamos los datos
  client.publish("infind/GRUPO2/datos/cocina",serializa_JSON2(misdatos).c_str() );
//Publicamos el estado de conexion con retain flag=true

  
    lastMsg = now;
  }


    if (interrupcion==true)
  {
    if (estado_int==LOW)
    {
      Serial.println("Se detectó una pulsación");
      if (ahora-lastInt < 500 && doblepulsacion==1)
      {
        doblepulsacion=2;
        
      }
      else
      {
      doblepulsacion=1;
      
      }
      
    }
    if (estado_int==HIGH)
    { 
        lastInt = now;
        if (ahora-penultima_int < 5000 && doblepulsacion==2)
       {
                if(ledboton2==0)
         {
                   Serial.println("Apagado 2");
                   ledboton2=1;
                   LED_dato=1023;
                   analogWrite(BUILTIN_LED,LED_dato); //Envio el valor al pin del led
                   LED_dato=100;
                   sprintf(msg2," {\"led\": %f} ",LED_dato);
                      //publica que he llegado a ese valor
                   client.publish("infind/GRUPO2/led/status/cocina",msg2 ); //Publico que he recibido el dato del led
     
        }
               else
        {
                  Serial.println("Encendido 2");
                  ledboton2=0;
                  LED_dato=0;
                  analogWrite(BUILTIN_LED,LED_dato); //Envio el valor al pin del led

                   sprintf(msg2," {\"led\": %f} ",LED_dato);
                      //publica que he llegado a ese valor
                   client.publish("infind/GRUPO2/led/status/cocina",msg2 ); //Publico que he recibido el dato del led
          
        }
       }
       else if ( ahora-penultima_int > 5000)
       {
        Serial.println("Actualizacion");
       }
       
    } 
    interrupcion=false;
    }

       if (ahora-lastInt > 500 && doblepulsacion==1) {
        if(ledboton==0)
        {
           Serial.println("Apagado");
           ledboton=1;
           doblepulsacion=0;
           digitalWrite(LED_Secundario, HIGH);
        }
        else
        {
          Serial.println("Encendido");
          ledboton=0;
          doblepulsacion=0;
          digitalWrite(LED_Secundario, LOW);
        }
       }

  }
  
