//  -----------------------[ INICIALIZACIÓN ]-----------------------
#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif
// Se incluyen y se definen los paquetes o expresiones para el correcto funcionamiento y envío/recepción de datos 
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
DHTesp dht;

// VCC
ADC_MODE(ADC_VCC);

// Datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("192.168.0.241")       // Dirección del servidor de actualización OTA 192.168.1.113
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Campo firmware para actualizar
#define HTTP_OTA_PORT         1880                     // Port of update server
                                                       // Nombre del firmware
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu"

#define OTA_URL "https://iot.ac.uma.es:1880/esp8266-ota/update"// Address of OTA update server
#define HTTP_OTA_VERSION   String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1)+".nodemcu"
String OTAfingerprint("5D 56 09 5C 5F 7B A4 3F 01 B7 22 31 D3 A7 DA A3 6E 10 2E 60"); // sustituir valor

//  ----------------------- FIN INICIALIZACIÓN -----------------------

// -----------------------[ ESTRUCTURAS JSON ]-----------------------
// Se definen tipos de datos para cada estructura JSON

//Estructura de json para publicar los datos globales
struct registro_datos {
  float   temperatura;
  float   humedad;
  float   vcc;
  unsigned long tiempo;
  long    rssi;
  String  ssid;
  String  mqtt_server;
  float   valor_LED;
  int     valor_SWITCH;
  float   lpg;
  float   co;
  float   humo;
  String  CHIPI;
  };
  
// Estructura de json para publicar conexión
  struct registro_conexion { 
    String  CHIPI;
    bool    conexion;
  };
  
  // Estructura de json para publicar actualización
  struct registro_actualizacion { 
    String  CHIPI;
    bool    actualizacion;
    String  origen;
  };
  
// Estructura de json para publicar datos del sensor LÁSER
  struct registro_distancia { 
    String  CHIPI;
    String  estado_sensor;
    float   distancia;
  };

// Estructura de json para publicar valor del LED (GPIO 2)
  struct registro_led {
    String  CHIPI;
    float   LED;
    String  origen;
  };

// Estructura de json para publicar valor del SWITCH (GPIO 16)
  struct registro_switch {
    String  CHIPI;
    int     SWITCH;
    String  origen;
  };
  
// Estructura de json para publicar valor de Peligrosidad (Sensor MQ2)
  struct registro_MQ2 {
    String CHIPI;
    int CalidadCO;
    int CalidadLPG;
    int CalidadHumo;
  };
  
//  ----------------------- FIN ESTRUCTURAS JSON -----------------------

// -----------------------[ DECLARACION  DE VARIABLES ]-----------------------
// Se declaran las variables que se usarán a lo largo del código, inicianizando algunas de ellas a valores específicos

//--------Wifi - MQTT--------
const char* ssid = "vodafoneAAR6W7"; // JAVI: ALFIL_PISOS // LEO: vodafoneAAR6W7 // 
const char* password = "Hn3fF6xXYFbgPJsH"; // JAVI: alfil2020 // LEO: Hn3fF6xXYFbgPJsH
const char* mqtt_server = "iot.ac.uma.es"; // Servidor MQTT común a todos los integrantes del grupo

WiFiClient espClient;
PubSubClient client(espClient);

// Se define el ancho del buffer para las estructuras json
//#define MSG_BUFFER_SIZE (50)
//const char* msg[MSG_BUFFER_SIZE];   // Para datos globales
//char msg_led[MSG_BUFFER_SIZE];      // Para valor de LED
//char msg_recDatos[MSG_BUFFER_SIZE]; // Para recogida de datos
//char msg_switch[MSG_BUFFER_SIZE];   // Para valor de SWITCH

//--------Interrupciones--------
int boton_flash=0;                // GPIO0 = boton flash
int estado_polling=HIGH;          // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
volatile int estado_int=HIGH;     // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW

unsigned long inicio_pulso_polling = 0; // inicialización a 0 del pulso

volatile unsigned long ahora;     // Se pondrá como global porque también se usará en el LOOP() - tiempo en que se da la actualización
volatile unsigned long penultima_int = 0; // Ayuda al envío de ultimo_dato antes de actualizarlo al valor actual
unsigned long ultima_int = 0;     // Para la última interrupción

volatile bool interrupcion=false; // Se usará esta variable para indicar cuándo se da una interrupción

//--------OTA--------
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);
unsigned long lastAct     = 0;        // Tiempo de última comprobación de actualización
//--------Interrupciones--------
unsigned long lastMsg   = 0;        // Último mensaje para datos enviado
volatile bool actualiza = false;  // Si actualiza=true, se buscan actualizaciones [loop()]
bool cuenta_pulsos      =true;          // A partir de él, se permite conocer si se han dado 1 o 2 interrupciones

unsigned long inicio = 0;         // Tiempo en que se produce una interrupción
int cont=0;

int LED_Secundario = 16;          //GPIO 16 
//int BUILTIN_LED  = 2;            // GPIO 2 - predefinido en 'Herramientas'
int estado_led2 =1;               // Inicialmente, ambos LEDs están encendidos: estado=1. En caso de estar apagados, estado=0
int estado_led16=1;

//--------PWM--------
volatile float LED_dato = 100;    // Valor del LED que se pretende alcanzar
float LED_max=100;                // Valor del LED que se pretende alcanzar
volatile float PWM;               // Valor del PWM
float LED_anterior = LED_dato;    // Registra el último valor del LED (!=0)

//--------Recogida de datos--------
volatile double TiempoRecogida=10;  // Recoger datos sensores (segundos)
volatile double TiempoLED     =10;       // Tiempo en que tarda en cambiar el LED un 1%
volatile int TiempoActualiza  = 60;  // Tiempo en comprobar actualizaciones (minutos)

//--------LASER--------
unsigned long lastMsgLASER = 0;     // último mensaje enviado desde sensor de distancia
float cambia_dist          = 0;
bool comprueba_laser       =false;
String nombre_casa=" ";
 
//--------Peligro--------
int CalidadCOa  =0;                 //Nivel de peligro de CO
int CalidadLPGa =0;                //Nivel de peligro de LPG: gas natural
int CalidadHumoa=0;               //Nivel de peligro de Humo

//--------Formalización TOPICS para envío MQTT en JSON--------
// Construimos estructura String
const String Num_Grupo="2";
const String topic_estructura="infind/GRUPO"+Num_Grupo+"/ESP"+String(ESP.getChipId());

// Topics (PUBLISH) en String
String topicS_global=topic_estructura+"/datos";
String topicS_conexion=topic_estructura+"/conexion";
String topicS_actualizacion=topic_estructura+"/actualizacion";
String topicS_laser=topic_estructura+"/puerta";
String topicS_led=topic_estructura+"/led/status";
String topicS_switch=topic_estructura+"/switch/status";
String topicS_mq2=topic_estructura+"/Calidad_Aire";

// Topics (SUBSCRIBE) en String
String topicS_FOTA=topic_estructura+"/FOTA";
String topicS_led_cmd=topic_estructura+"/led/cmd";
String topicS_switch_cmd=topic_estructura+"/switch/cmd";
String topicS_config=topic_estructura+"/config/cmd";
String topicS_asignacion=topic_estructura+"/asignacion";

// Topics (PUBLISH) en char*
const char* topic_global=topicS_global.c_str();
const char* topic_conexion=topicS_conexion.c_str();
const char* topic_actualizacion=topicS_actualizacion.c_str();
const char* topic_laser=topicS_laser.c_str();
const char* topic_led=topicS_led.c_str();
const char* topic_switch=topicS_switch.c_str();
const char* topic_mq2=topicS_mq2.c_str();

// Topics (SUBSCRIBE) en char*
const char* topic_FOTA=topicS_FOTA.c_str();
const char* topic_led_cmd=topicS_led_cmd.c_str();
const char* topic_switch_cmd=topicS_switch_cmd.c_str();
const char* topic_config=topicS_config.c_str();
const char* topic_asignacion=topicS_asignacion.c_str();


//----------------------- FIN DECLARACIÓN DE VARIABLES -----------------------

// -----------------------[ INTERRUPCIONES ]-----------------------
//Pulsación del botón flash, permite encender o apagar el GPIO 2 y comprobar actualizaciones

// Rutina de Tratamiento de la Interrupcion (RTI)
ICACHE_RAM_ATTR void RTI() {
  
  int lectura=digitalRead(boton_flash); //Se toma el valor en alta o baja del botón flash
  interrupcion=true;                    // Inicialmente, salta la Interrupción por lo que se activa su variable correspondiente
  ahora= millis();                      // Se toma el valor actal (ms)
  
  // Para eliminar rebotes:
  if(lectura==estado_int || ahora-ultima_int<50) return; // Filtro anti rebotes 50ms
  if(lectura==LOW)                      // En caso de haber pulsado el botón flash
  { 
   estado_int=LOW; 
  }
  else                                  // En caso de haber soltado el botón flash
  {
   estado_int=HIGH; 
   penultima_int=ultima_int;            // Se toma el valor del tiempo en que se dio la penúltima interrupción 
  }                                     // para que se tenga constancia de cuándo se ha soltado el botón
  ultima_int = ahora; 
}
// ----------------------- FIN INTERRUPCIONES -----------------------

//-----------------------[ CONVERSIÓN A STRING----ESTRUCTURAS JSON ]-----------------------
//Esta funcion nos convierte a un String todos los datos globales para ser interpretados por Json
String serializa_JSONGlobal (struct registro_datos datos_globales)
{
  JSONVar jsonRoot;
  JSONVar DHT11;
  JSONVar Wifi;
  JSONVar MQ2;
  JSONVar ANALOG;
  String jsonString;
  
  DHT11["temp"] = datos_globales.temperatura;
  DHT11["hum"]  = datos_globales.humedad;
  Wifi["ssid"]  = datos_globales.ssid;
  Wifi["ip"]    = datos_globales.mqtt_server;
  Wifi["rssi"]  = datos_globales.rssi;
  MQ2["LPG"]    = datos_globales.lpg;
  MQ2["CO"]     = datos_globales.co; 
  MQ2["HUMO"]   = datos_globales.humo;
  
  jsonRoot["CHIPID"]=datos_globales.CHIPI="ESP"+String(ESP.getChipId()); // ID del Chip
  jsonRoot["Uptime"]= datos_globales.tiempo;
  jsonRoot["vcc"] = datos_globales.vcc/1000.;
  jsonRoot["DHT11"]= DHT11;
  jsonRoot["Wifi"]=Wifi;
  jsonRoot["LED"]=datos_globales.valor_LED;
  jsonRoot["SWITCH"]=datos_globales.valor_SWITCH;
  jsonRoot["MQ-2"]=MQ2;

  return JSON.stringify(jsonRoot);
}
// -----------
// Función para la conexion
String serializa_JSON (struct registro_conexion estado)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["CHIPID"] = estado.CHIPI="ESP"+String(ESP.getChipId());
  jsonRoot["online"] = estado.conexion;
  
  return JSON.stringify(jsonRoot);
}
// -----------

// Función para la actualización
String serializa_JSONact (struct registro_actualizacion estado_act)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;

  jsonRoot["CHIPID"]        = estado_act.CHIPI="ESP"+String(ESP.getChipId());
  jsonRoot["actualizacion"] = estado_act.actualizacion;
  jsonRoot["origen"]        = estado_act.origen;
  
  return JSON.stringify(jsonRoot);
}
// -----------

// Función para la distancia LASER
String serializa_JSONdistist (struct registro_distancia laser)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;

  jsonRoot["CHIPID"]  = laser.CHIPI="ESP"+String(ESP.getChipId());
  jsonRoot["Estado"]  = laser.estado_sensor;
  jsonRoot["VL53L0X"] = laser.distancia;
  
  return JSON.stringify(jsonRoot);
}
// -----------

// Función para el valor del LED
String serializa_JSONLED (struct registro_led valor_LED)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["CHIPID"] = valor_LED.CHIPI="ESP"+String(ESP.getChipId());     
  jsonRoot["led"]    = valor_LED.LED;        // valor de LED
  jsonRoot["origen"] = valor_LED.origen;    // Indica de dónde proviene el valor (si se ha estimulado por pulsación o por mqtt)
  
  return JSON.stringify(jsonRoot);
}
// -----------

// Función para el valor del SWITCH
String serializa_JSONSWITCH (struct registro_switch valor_SWITCH)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["CHIPID"] = valor_SWITCH.CHIPI="ESP"+String(ESP.getChipId()); 
  jsonRoot["switch"] = valor_SWITCH.SWITCH; // valor del SWITCH
  jsonRoot["origen"] = valor_SWITCH.origen; 
  
  return JSON.stringify(jsonRoot);
}
// -----------

// Función para el valor de la Calidad del aire
String serializa_JSONMQ2(struct registro_MQ2 calidad_aire)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  
  jsonRoot["CHIPI"] = calidad_aire.CHIPI="ESP"+String(ESP.getChipId());
  jsonRoot["CO"]    = calidad_aire.CalidadCO; 
  jsonRoot["LPG"]   = calidad_aire.CalidadLPG; 
  jsonRoot["Humo"]  = calidad_aire.CalidadHumo; 
  
  return JSON.stringify(jsonRoot);
}
//----------------------- FIN CONVERSIÓN A STRING----ESTRUCTURAS JSON ----------------------- 

//-----------------------[ CONEXIÓN A INTERNET ]-----------------------
//Setup de la conexion a internet
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
//----------------------- FIN CONEXIÓN A INTERNET ----------------------- 

//-----------------------[ CONEXIÓN MQTT ]-----------------------
//Bucle de conexion MQTT

void reconnect() {
  struct registro_conexion estado;          // Declaramos la estructura para conexión (json)
  struct registro_actualizacion estado_act; // Declaramos la estructura para actualización (json)
  
  // Permanecemos en el bucle hasta que nos reconectemos
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Se crea un cliente el ID propio de cada chip
    String clientId = "ESP8266Client-";
    clientId += String(ESP.getChipId()) ;
    
    // Esperamos a conexión
    estado.conexion=false;                  // Inicialmente, no hemos establecido conexión

    // Enviamos por el topic de actualizaciones que no se ha buscado actualización aún
    estado_act.actualizacion=false;         // No se ha actualizado aún
    estado_act.origen="";
    client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str());
    // Conexión al servidor y últimas voluntades
    if (client.connect(clientId.c_str(),"infind","zancudo",topic_conexion,0,true,serializa_JSON(estado).c_str())) {
      Serial.println("connected"); // Estaremos conectados. Usuario: infind. Contraseña: zancudo. 
      estado.CHIPI=String(ESP.getChipId());
      estado.conexion = true;
      client.publish(topic_conexion,serializa_JSON(estado).c_str(),true );
      
      // Subscripción a los distintos topics
      client.subscribe(topic_FOTA);              //Me suscribo al topic de comprobación de actualizaciones
      client.subscribe(topic_led_cmd);     //Me suscribo al topic del estado del GPIO 2
      client.subscribe(topic_switch_cmd);  //Me suscribo al topic del estado del GPIO 16
      client.subscribe(topic_config);  //Me suscribo al topic de configuración
      client.subscribe(topic_asignacion);
     }
     else { // En caso de no haber establecido conexión, reintenta a los 5 segundos
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//----------------------- FIN CONEXIÓN MQTT ----------------------- 

//-----------------------[ CALLBACK ]-----------------------
// Funciones que estudian los 'inTopic'

void callback(char* topic, byte* payload, unsigned int length) {
  char* mensaje=(char*)malloc(length+1);     // Reservo memoria para copia del mensaje
  strncpy(mensaje,(char*)payload,length);    // Copio el mensaje en cadena de caracteres
  
  struct registro_led valor_LED;             // Declaramos registro json para valores del LED
  struct registro_switch valor_SWITCH;       // Declaramos registro json para valores del SWITCH
  // Comprobación ------Asignación nombre------
  if(strcmp(topic,topic_asignacion)==0) // Se estudia el valor del topic de Actualización
    {
      StaticJsonDocument<512> root;               // El tamaño tiene que ser adecuado para el mensaje
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(root, mensaje);

      // Compruebo si no hubo error
      if (error) {
        Serial.print("Error deserializeJson() failed: ");
        Serial.println(error.c_str());
      }
      else // En caso de no haber error:
      {
        nombre_casa = root["asignacion"];            // Buscamos actualizaciones
        
        Serial.println(nombre_casa);
      }
      free(mensaje); // libero memoria
   }
  // Comprobación ------Actualizaciones------
  if(strcmp(topic,topic_FOTA)==0) // Se estudia el valor del topic de Actualización
    {
      Serial.println(actualiza);
      StaticJsonDocument<512> root;               // El tamaño tiene que ser adecuado para el mensaje
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(root, mensaje);

      // Compruebo si no hubo error
      if (error) {
        Serial.print("Error deserializeJson() failed: ");
        Serial.println(error.c_str());
      }
      else // En caso de no haber error:
      {
        actualiza = root["actualiza"];            // Buscamos actualizaciones
        Serial.println(actualiza);
      }
      free(mensaje); // libero memoria
   }
   
  // Comprobación ------SWITCH------
  if(strcmp(topic,topic_switch_cmd)==0)
  {
    StaticJsonDocument<512> root;
    // Deserializa el documento JSON
    DeserializationError error = deserializeJson(root, mensaje);

    // Compruebo si hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else  // En caso de no haber error, se hace la secuencia propuesta:
    {
      double switch_max = root["level"];  // Se toma el valor del dato de entrada
      
      if (switch_max == 0)                // Switch apagado
      {
        estado_led16 = 0;
        digitalWrite(LED_Secundario,HIGH);
      }
      else if (switch_max == 1)           // Switch encendido
      {
        estado_led16 = 1;
        digitalWrite(LED_Secundario,LOW);
      }
       // Publicamos por el topic correspondiente
      valor_SWITCH.SWITCH = estado_led16;
      valor_SWITCH.origen = "mqtt";
      //Publicamos los datos
      client.publish(topic_switch,serializa_JSONSWITCH(valor_SWITCH).c_str());         //Publico que he recibido el dato del switch
    }
    free(mensaje); // libero memoria
  }
  
  // Comprobación ------LED-----
  if(strcmp(topic,topic_led_cmd)==0)
  {
    StaticJsonDocument<512> root;
    // Deserializa el documento JSON
    DeserializationError error = deserializeJson(root, mensaje);

    // Compruebo si hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else                              // En caso de no haber error, se hace la secuencia propuesta:
    {
     LED_max = root["level"];         // Se toma el valor del dato de entrada
     
     double subirled=LED_dato;        // Iniciamos variable
     
     // Aumentamos intensidad del LED
     while(subirled<LED_max)          // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
     {
      PWM = 1023*(1-subirled/100);     // Se actualiza la intensidad del LED a través de PWM
      analogWrite(BUILTIN_LED,PWM);    //Envio el valor al pin del led
     
      // Publicamos por el topic correspondiente
      valor_LED.LED    = subirled;
      valor_LED.origen = "mqtt"; 
      client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() ); 
     
      subirled++;
      delay(TiempoLED);                // Se espera el tiempo exigido por el usuario
     }

     // Disminuimos intensidad del LED
     while(subirled>LED_max)          // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
     {  
     PWM = 1023*(1-subirled/100); 
     analogWrite(BUILTIN_LED,PWM); 
     
     // Publicamos por el topic correspondiente
     valor_LED.LED    = subirled;
     valor_LED.origen = "mqtt"; 
     client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() ); 
     
     subirled--;
     delay(TiempoLED);
     }
     
     PWM = 1023*(1-LED_max/100);    //Finalmente, se alcanza el valor de LED propuesto por el usuario
     analogWrite(BUILTIN_LED,PWM);  //Envio el valor al pin del led
     
     LED_dato = subirled;
     
     if (LED_dato != 0)
      LED_anterior = LED_dato;
      
     // Publicamos por el topic correspondiente
     valor_LED.LED    = LED_dato;
     valor_LED.origen = "mqtt"; 
     client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() );
     
    if (LED_dato==0) // Estudio del estado del LED (apagado o encendido)
    {
      Serial.println("GPIO 2 APAGADO");
      estado_led2=0;
    }
    else
    {
      Serial.println("GPIO 2 ENCENDIDO");
      estado_led2=1;
    }
  }
  free(mensaje); // libero memoria
  }
  
    // Comprobación ------Recogida de Datos-----
    if(strcmp(topic,topic_config)==0) // Se estudia el estado del topic específico
    {
      StaticJsonDocument<512> root;
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(root, mensaje);

      // Compruebo si no hubo error
      if (error) {
        Serial.print("Error deserializeJson() failed: ");
        Serial.println(error.c_str());
      }
      else // En caso de no haber error:
      {
        double valorDatos     = root["envia"];      // Se toman los valores corresponientes al tiempo del envío de mensajes (s)
        double valorActualiza = root["actualiza"];  // Se toma el tiempo de comprobación de actualizaciones (min)
        double valorVelocidad = root["velocidad"];  // Se toma la velocidad de cambio de salida PWM (ms)
        float valorLED        = root["LED"];        // Configura si salida está activada/encendida a nivel alto(1) o bajo(0). null: se mantiene configuración
        int valorSWITCH       = root["SWITCH"];     // Mismas condiciones que para valorLED

        // -- Envia -- 
        if (root["envia"] == nullptr)                 // Valor recibido sin valor, mantenemos el último valor
        {
        }
        else
        {
          TiempoRecogida=valorDatos;
        }
        // -- Actualiza -- 
        if (root["envia"] == nullptr)                 // Valor recibido sin valor, mantenemos el último valor
        {
        }
        else
        {
          TiempoActualiza=valorActualiza;             // Asignamos a una variable global ya que se usará en otros bucles externos al 'Callback'
        }

        // -- Velocidad -- 
        if (root["envia"] == nullptr)                 // Valor recibido sin valor, mantenemos el último valor
        {
        }
        else
        {
          TiempoLED=valorVelocidad;
        }
        // -- LED -- 
        if (root["LED"] == nullptr)                 // Valor recibido sin valor, mantenemos el último valor
        {
          Serial.println("Valor LED NULL");
          //analogWrite(BUILTIN_LED,LED_dato);
        }
        else if (valorLED == 0)                     // Valor recibido = 0, apagamos LED
        {
          digitalWrite(LED_Secundario,HIGH);
          estado_led2 = 0;
          LED_dato = 0;
          
          // Publicamos por el topic correspondiente
          valor_LED.LED    = LED_dato;
          valor_LED.origen = "mqtt"; 
          client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() );
        }
        else if (valorLED == 1)                     // Valor recibido = 1, encendemos al valor máximo el LED
        {
          digitalWrite(LED_Secundario,LOW);
          estado_led2=1;
          LED_dato = 100;
          
          // Publicamos por el topic correspondiente
          valor_LED.LED    = LED_dato;
          valor_LED.origen = "mqtt"; 
          client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() );
        }
        
        // -- SWITCH --
        if (root["SWITCH"] == nullptr)              // Valor recibido sin valor, mantenemos el último valor
        {
          // Mantiene valor anterior
        }
        else if (valorSWITCH == 0)                  // Valor recibido = 0, apagamos SWITCH
        {
          digitalWrite(LED_Secundario,HIGH);
          estado_led16 = 0;
          
          // Publicamos por el topic correspondiente
          valor_SWITCH.SWITCH = estado_led16;
          valor_SWITCH.origen = "mqtt";
          //Publicamos los datos
          client.publish(topic_switch,serializa_JSONSWITCH(valor_SWITCH).c_str());  //Publico que he recibido el dato del switch
        }
        else if (valorSWITCH == 1)                  // Valor recibido = 1, encendemos SWITCH
        {
          digitalWrite(LED_Secundario,LOW);
          estado_led16=1;
          
          // Publicamos por el topic correspondiente
          valor_SWITCH.SWITCH = estado_led16;
          valor_SWITCH.origen = "mqtt"; 
          client.publish(topic_switch,serializa_JSONSWITCH(valor_SWITCH).c_str());
        }
        
      }
      free(mensaje); // libero memoria
  }
  
}
//----------------------- FIN CALLBACK ----------------------- 

//-----------------------[ SETUP ]-----------------------
void setup() {
  struct registro_actualizacion estado_act; // Declaramos la estructura para actualización (json)
  struct registro_conexion estado;          // Declaramos la estructura para conexión (json)
  
  pinMode(BUILTIN_LED, OUTPUT);             // Inicializamos el pin BUILTIN_LED como salida
  pinMode(LED_Secundario, OUTPUT);          // Inicializamos el pin LED_Secundario como salida
  digitalWrite(LED_Secundario, LOW);        // Encendemos inicialmente GPIO 2
  analogWrite(BUILTIN_LED, 0);              // Encendemos inicilamente GPIO 16
  Serial.begin(9600);
  setup_wifi();
  
  // Hacemos saber que la actualización se debe al arrancar el dispositivo
  estado_act.actualizacion = true;
  estado_act.origen        = "inicio";
  client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str(),true);
  
  // OTA -> Actualizamos al empezar
      Serial.println( "--------------------------" );
      Serial.println( "Comprobando actualización:" );
      Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH); // Se comprueba la actualización
      Serial.println( "--------------------------" );  
      
      estado.conexion=false;                                                                         // Se desconecta mientras se realiza la actualización
      client.publish(topic_conexion,serializa_JSON(estado).c_str(),true);      // Publicamos por el topic de conexión que nos hemos desconectado debido a la actualización
      
      estado_act.actualizacion=false;
      client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str(),true);
      
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
          
          actualiza=true;
          
          estado_act.actualizacion=actualiza;
          client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str());
          break;
        case HTTP_UPDATE_OK:
          Serial.println(F(" OK"));
          break;
        } 
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);

  client.setBufferSize(512);            //Ampliamos el tamaño del buffer
  char cadena[512];
  dht.setup(14, DHTesp::DHT11);          // Connect DHT sensor to GPIO 14

  //FLASH
   pinMode(boton_flash, INPUT_PULLUP);  // Inicializamos el pin LED_Secundario como entrada
   
  // Activamos la interrupción
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);
 
  //MQ-2
  mq2.begin();
}
//-----------------------FIN SETUP ----------------------- 

//-----------------------[ OTA ]-----------------------
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
//----------------------- FIN OTA ----------------------- 

//-----------------------[ LOOP ]-----------------------
void loop() {
  
    // CONEXIÓN
  String mensaje_conexion;
  if (!client.connected()) {
    reconnect();
  }
    
  client.loop();
  
  // Declaramos inicialmente estructuras json que se emplearán
  struct registro_conexion estado;          // Declaramos la estructura para conexión (json)
  struct registro_datos datos_globales;    // json para datos globales
  struct registro_led valor_LED;           // Declaramos registro json para valores del LED
  struct registro_distancia laser;         // json para sensor laser
  struct registro_MQ2 calidad_aire;        // json para sensor de calidad de aire 
  struct registro_actualizacion estado_act; // json para confirmar búsqueda de actualizaciones

  unsigned long lastMsg=0;
  unsigned long lastPeligro = 0;        // Tiempo de último envío de datos de calidad de aire
  unsigned long now=millis();
  
    // INTERRUPCIONES
  if (interrupcion==true)               // En caso de que se active la interrupción
  {
    if (cuenta_pulsos==true) 
    {
      inicio=ahora;                     // Estudiamos el tiempo en que se produce la interrupción
      cuenta_pulsos=false;
    }
    if (estado_int==LOW)                // Si el botón está pulsado, se toma el registro del tiempo en que se produce
    {
      Serial.print("Se detectó una pulsación en millis = ");
      Serial.print(ahora);
      Serial.println(" ms");
    }
    if (estado_int==HIGH)               // Cuando se suelta el botón, se tomar el tiempo y se hace la diferencia, conociendo el tiempo que se mantiene pulsado
    {
      Serial.print("Terminó la pulsación, duración = ");
      Serial.print(ahora-penultima_int);
      Serial.println(" ms");
      cont++;                          // Se ha dado un pulso 
      if (ahora-penultima_int > 2000)  // Si NO haya actualización, se estudian las pulsaciones
      {
        // Si se mantiene +2seg pulsado, se habilita la señal que permite actualizar
        actualiza=true;
        cont=0;                        // Para evitar encender o apagar el LED, se resetea el contador de pulsos
        
        // Hacemos saber que la actualización se debe por el pulsador flash
        estado_act.actualizacion=true;
        estado_act.origen="pulsador";
        client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str(),true);
      }
    }
    interrupcion=false;                // Desabilitamos la interrupción hasta que vuelva a haber una llamada a la misma
  }
    now=millis();
    
      if (now-inicio>650)                  // Cuando pasen 0.65 seg tras la interrupción
      {
        if (cont==2)                       // Si se han contado 2 pulsos -> encendemos al nivel máximo el GPIO 16
        {
          Serial.println("DOBLE PULSACION");
          analogWrite(BUILTIN_LED,0);      // Encendemos al nivel máximo (activo al nivel bajo como PWM
          Serial.println("GPIO 2 ENCENDIDO al MÁX");
          estado_led2=1;
          LED_dato = 100;                  // Registramos el valor del LED
          LED_anterior = LED_dato;         // Registramos el último valor encendido
          
          cont=0;                          // Reseteamos la cuenta de pulsos
          cuenta_pulsos=true;              // Se activa la contación de pulsos a partir del nº de interrupciones

          // Publicamos por el topic correspondiente
          valor_LED.LED=LED_dato;
          valor_LED.origen="pulsador"; 
          client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() );
        }
        if (cont==1) // Si solo se ha contado 1 pulsación, encendemos o apagamos el LED principal (GPIO 16)
        {
        Serial.println("UNA PULSACION");

        if (estado_led2==1)                // Si estaba encendido, apagamos
          {
            analogWrite(BUILTIN_LED,1023);
            Serial.println("GPIO 2 APAGADO");
            estado_led2 = 0;
            LED_dato = 0;
            
          }
          else                              // Si estaba apagado, encendemos al nivel anterior
          { 
            PWM = 1023*(1-LED_anterior/100); 
            analogWrite(BUILTIN_LED,PWM);
            Serial.println("GPIO 2 ENCENDIDO AL NIVEL ANTERIOR");
            estado_led2 = 1;
            LED_dato = LED_anterior;
            
          }
          cont=0;                           // Reseteamos la cuenta
          cuenta_pulsos=true;
          
          // Publicamos por el topic correspondiente
          valor_LED.LED=LED_dato;
          valor_LED.origen="pulsador"; 
          client.publish(topic_led,serializa_JSONLED(valor_LED).c_str());
        }
        else if (cont>2)                    // Si se cuentan más de 2 pulsos, se hace saber que no es válido
        {
          Serial.println("MAS DE 2 PULSACIONES, CÓDIGO NO VÁLIDO");

          cont=0;
          cuenta_pulsos=true;
        }
    delay(10);
    }
  
  now=millis();
  // DATOS GLOBALES
   if (now - lastMsg > TiempoRecogida*1000) {
  
  
   //Las variables que vamos a meter en nuestra estructura datos
   datos_globales.ssid= WiFi.localIP().toString();
   datos_globales.mqtt_server= WiFi.SSID(); 
   datos_globales.vcc = ESP.getVcc();
   datos_globales.rssi = WiFi.RSSI();
   datos_globales.temperatura = round(dht.getTemperature());
   datos_globales.humedad = dht.getHumidity();
   datos_globales.lpg =mq2.readLPG();
   datos_globales.co= mq2.readCO();
   datos_globales.humo = mq2.readSmoke();
   datos_globales.tiempo = millis();
   datos_globales.valor_LED = LED_dato;
   datos_globales.valor_SWITCH = estado_led16;
   datos_globales.CHIPI = String(ESP.getChipId());  
  Serial.println("JAJA");
  client.publish(topic_global,serializa_JSONGlobal(datos_globales).c_str() );
  //Publicamos el estado de conexion con retain flag=true
  
  
    lastMsg = now;
  }
  
  
  now=millis();
  // ACTUALIZACIÓN
  if (now - lastAct > TiempoActualiza*60000 && TiempoActualiza != 0)
  {
    actualiza=true;                           // En caso de que se cumpla el Tiempo deseado para buscar actualizaciones, se habilita la señal actualiza, tomando el valor true

    // Hacemos saber que la actualización se debe por vencimiento de tiempo
    estado_act.actualizacion=actualiza;
    estado_act.origen="mqtt";
    client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str(),true);
  }
  
  if (actualiza==true )                       // Mantenemos pulsado +2s o vence el tiempo de actualización: ACTUALIZAMOS! 
    {
      // OTA
      Serial.println( "--------------------------" );
      Serial.println( "Comprobando actualización:" );
      Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH); // Se comprueba la actualización
      Serial.println( "--------------------------" );  
      
      estado.conexion=false;                                                                         // Se desconecta mientras se realiza la actualización
      client.publish(topic_conexion,serializa_JSON(estado).c_str(),true);      // Publicamos por el topic de conexión que nos hemos desconectado debido a la actualización
      
      actualiza=false;                        // Se 'resetea' la variable booleana que habilita la actualización
      estado_act.actualizacion=actualiza;
      client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str());
      
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
          actualiza=true;
          
          estado_act.actualizacion=actualiza;
          client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str());
          
          break;
        case HTTP_UPDATE_OK:
          Serial.println(F(" OK"));
          break;
        }
      lastAct = now; 
    }
/*
  // LASER
  if (nombre_casa=="salon") {
  if (comprueba_laser == false)                   // Inicialmente, se estudia estado el sensor
  {
    if (!lox.begin()) {
      Serial.println(F("Error al iniciar VL53L0X"));
      laser.estado_sensor = "Fallo al iniciar";   // Se hace saber que ha dado fallo al inicializar
      laser.distancia = -1;
      delay(5000);
    }
    else
    {
      comprueba_laser=true;
      laser.estado_sensor = "VL53L0X activo";     // Se hace saber que el sensor se ha inicializado correctamente
      Serial.println("Iniciado VL53L0X");
      laser.distancia = 0;
    }
    // Publicamos la distancia por el topic correspondiente
    client.publish(topic_laser,serializa_JSONdistist(laser).c_str() ); 
  }
  else                                            // Cuando esté activo el sensor, se podrá leer datos del sensor
  {
    VL53L0X_RangingMeasurementData_t measure;
  
    // Leyendo sensor...
    lox.rangingTest(&measure, false);             // Si se pasa true como parametro, muestra por puerto serie datos de debug
    float medida_laser = measure.RangeMilliMeter;
    
    laser.distancia = medida_laser;
    laser.estado_sensor = "VL53L0X iniciado";
    
    // Cuando se salga de un rango específico, se toma una nueva medida y se hace saber al usuario
    if (cambia_dist >= medida_laser+100 || cambia_dist<=medida_laser-100) 
    {
    Serial.print("Distancia (mm): ");
    Serial.println(measure.RangeMilliMeter);
    
    // Publicamos la distancia por el topic correspondiente
    client.publish(topic_laser,serializa_JSONdistist(laser).c_str() ); 
    
    cambia_dist=medida_laser;                   // Actualizamos la distancia 
  }
  }
  }*/

    
    // PELIGROSIDAD MQ2
    // Se analizan los niveles y se determina la peligrosidad respectiva
    
    // Se analizan los niveles de CO 
    if (mq2.readCO()>=1.2)
    {
      CalidadCOa=3;
    }
    else if (mq2.readCO()>=0.9 && mq2.readCO()<1)
    {
      CalidadCOa=2;
    }
    else if (mq2.readCO()>=0.8 && mq2.readCO()<0.9)
    {
      CalidadCOa=1;
    }
    else
    {
      CalidadCOa=0;
    }

    //Se analizan los niveles de LPG
    if (mq2.readLPG()>=1.2)
    {
      CalidadLPGa=3;
    }
    else if (mq2.readLPG()>=0.9 && mq2.readLPG()<1)
    {
      CalidadLPGa=2;
    }
    else if (mq2.readLPG()>=0.8 && mq2.readLPG()<0.9)
    {
      CalidadLPGa=1;
    }
    else
    {
      CalidadLPGa=0;
    }

    // Se analizan los niveles de LPG
    if (mq2.readSmoke()>=1.2)
    {
      CalidadHumoa=3;
    }
    else if (mq2.readSmoke()>=0.9 && mq2.readSmoke()<1)
    {
      CalidadHumoa=2;
    }
    else if (mq2.readSmoke()>=0.8 && mq2.readSmoke()<0.9)
    {
      CalidadHumoa=1;
    }
    else
    {
      CalidadHumoa=0;
    }
    now=millis();
    // En caso de haber haber peligro en algún valor, se hace saber al usuario
    if ((CalidadHumoa>0 || CalidadLPGa>0 || CalidadCOa>0)||(now-lastPeligro>20000))
    {
      //Valores de Peligrosidad
      calidad_aire.CalidadCO=CalidadCOa; 
      calidad_aire.CalidadLPG=CalidadLPGa; 
      calidad_aire.CalidadHumo=CalidadHumoa; 
  
      client.publish(topic_mq2,serializa_JSONMQ2(calidad_aire).c_str() );
      //Publicamos el estado de conexion con retain flag=true
      lastPeligro=now;
    }
  delay(10);
  
}
//----------------------- FIN LOOP ----------------------- 
