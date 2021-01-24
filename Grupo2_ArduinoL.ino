//  -----------------------[ INICIALIZACIÓN ]-----------------------
#ifdef ESP32
#pragma message(THIS PROYECT IS FOR ESP8266 ONLY!)
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

// Peligro AIRE
#include <MQ2Lib.h>
int pin = 0; //Pin al que se conectará de forma analógica
MQ2 mq2(pin, true); // Habilitamos el pin

// DHT
#include "DHTesp.h"
DHTesp dht;

// VCC
ADC_MODE(ADC_VCC);

// Datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("192.168.0.241")       // Dirección del servidor de actualización OTA, aunque el real sería el definido como OTA_URL
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Campo firmware para actualizar
#define HTTP_OTA_PORT         1880                     // Puerto del servidor de actualizaciones
                                                       // Nombre del firmware
                                                    
#define OTA_URL "https://iot.ac.uma.es:1880/esp8266-ota/update"// Dirección del servidor de actualizaciones OTA
#define HTTP_OTA_VERSION   String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1)+".nodemcu"
String OTAfingerprint("5D 56 09 5C 5F 7B A4 3F 01 B7 22 31 D3 A7 DA A3 6E 10 2E 60"); // sustituir valor por la firma del servidor

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
    int PeligroCO;
    int PeligroLPG;
    int PeligroHumo;
  };
  
//  ----------------------- FIN ESTRUCTURAS JSON -----------------------

// -----------------------[ DECLARACION  DE VARIABLES ]-----------------------
// Se declaran las variables que se usarán a lo largo del código, inicianizando algunas de ellas a valores específicos

//--------Wifi - MQTT--------

const char* ssid="vodafoneAAR6W7";          // INTRODUCIR SSID PROPIO DE CADA USUARIO
const char* password="Hn3fF6xXYFbgPJsH";    // INTRODUCIR PASSWORD PROPIA DE CADA USUARIO
const char* mqtt_server = "iot.ac.uma.es";  // Servidor MQTT común a todos los integrantes del grupo

WiFiClient espClient;
PubSubClient client(espClient);

//--------Últimos mensajes enviados--------
unsigned long lastMsg       = 0;        // Último mensaje para datos enviado
unsigned long lastPeligro   = 0;        // Tiempo de último envío de datos de Peligro de aire
unsigned long lastMsgLASER  = 0;        // Último mensaje enviado desde sensor de distancia
unsigned long lastLaser     = 0;        // Tiempo de último intento de inicialización
unsigned long lastAct       = 0;        // Tiempo de última comprobación de actualización


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
String origen_act;

//--------Interrupciones--------
volatile bool actualiza = false;  // Si actualiza=true, se buscan actualizaciones [loop()]
bool cuenta_pulsos      =true;    // A partir de él, se permite conocer si se han dado 1 o 2 interrupciones
unsigned long inicio = 0;         // Tiempo en que se produce una interrupción
int cont=0;

int LED_Secundario = 16;          // GPIO 16 
//int BUILTIN_LED  = 2;           // GPIO 2 - predefinido en 'Herramientas'
int estado_led2 =1;               // Inicialmente, ambos LEDs están encendidos: estado=1. En caso de estar apagados, estado=0
int estado_led16=1;

//--------PWM--------
String origen_LED;
bool cambia_PWM=false;            // En el momento en que valga 'true' se cambiará el nivel del LED en base a PWM
bool logica_PN=false;             // En caso de tomar valor 'false' implica lógia negativa // 'true' para lógica positiva
volatile float LED_dato = 100;    // Valor del LED que se pretende alcanzar
float LED_max=100;                // Valor del LED que se pretende alcanzar
float LED_anterior = LED_dato;    // Registra el último valor del LED (!=0)

//--------Recogida de datos--------
float TiempoRecogida  = 300;      // Recoger datos sensores (segundos): inicialmente en 5 min = 300 s
float TiempoLED       = 10;       // Tiempo en que tarda en cambiar el LED un 1%: inicialmente en 10 ms
int  TiempoActualiza  = 60;       // Tiempo en comprobar actualizaciones (minutos): inicialmente en 60 min = 1 hora
bool estado_conex     = false;

//--------Asignación dependencia--------
String nombre_casa;               // Asigna un nombre en función del CHIP ID (una estancia de la casa domotizada)

//--------LASER--------
int cambia_dist       = 0;        // Evita enviar continuamente los datos tomados del sensor láser
bool comprueba_laser  = false;    // En caso de estar en true, comprueba inicialización del sensor
float estado_laser;               // Medida tomada por el láser (-1 en caso de dar fallo por inicialización)

//--------Peligro--------
int PeligroCOa  =0;               // Nivel de peligro de CO
int PeligroLPGa =0;               // Nivel de peligro de LPG: gas natural
int PeligroHumoa=0;               // Nivel de peligro de Humo

//--------Formalización TOPICS para envío MQTT en JSON--------
// Construimos estructura String
const String Num_Grupo="2";       // Número de grupo para el Proyecto Final 
const String topic_estructura="infind/GRUPO"+Num_Grupo+"/ESP"+String(ESP.getChipId()); 

// Topics (PUBLISH) en String: se añaden terminaciones a la estructura global
String topicS_global=topic_estructura+"/datos";
String topicS_conexion=topic_estructura+"/conexion";
String topicS_actualizacion=topic_estructura+"/actualizacion";
String topicS_laser=topic_estructura+"/puerta";
String topicS_led=topic_estructura+"/led/status";
String topicS_switch=topic_estructura+"/switch/status";
String topicS_mq2=topic_estructura+"/Peligro";

// Topics (SUBSCRIBE) en String
String topicS_FOTA=topic_estructura+"/FOTA";
String topicS_led_cmd=topic_estructura+"/led/cmd";
String topicS_switch_cmd=topic_estructura+"/switch/cmd";
String topicS_config=topic_estructura+"/config/cmd";
String topicS_tipo_logica=topic_estructura+"/logica";

// Topics (PUBLISH) en char*: se transforma de String -> char* para ser pasados como Topics por MQTT 
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
const char* topic_tipo_logica=topicS_tipo_logica.c_str();

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
  JSONVar ANALOG;
  String jsonString;
  
  DHT11["temp"] = datos_globales.temperatura*10/10.0;     // Se realiza esta operación para contar con 1 único decimal
  DHT11["hum"]  = datos_globales.humedad;
  Wifi["SSId"]  = datos_globales.ssid;
  Wifi["IP"]    = datos_globales.mqtt_server;
  Wifi["RSSI"]  = datos_globales.rssi;

  jsonRoot["CHIPID"] = datos_globales.CHIPI="ESP"+String(ESP.getChipId()); // ID del Chip
  jsonRoot["Uptime"] = datos_globales.tiempo;
  jsonRoot["Vcc"]    = datos_globales.vcc/1000.;
  jsonRoot["DHT11"]  = DHT11;
  jsonRoot["LED"]    = datos_globales.valor_LED;
  jsonRoot["SWITCH"] = datos_globales.valor_SWITCH;
  jsonRoot["Wifi"]   = Wifi;

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
  jsonRoot["led"]    = valor_LED.LED;       // valor de LED
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

// Función para el valor de la Peligro del aire
String serializa_JSONMQ2(struct registro_MQ2 peligro_aire)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  
  jsonRoot["CHIPI"] = peligro_aire.CHIPI="ESP"+String(ESP.getChipId());
  jsonRoot["CO"]    = peligro_aire.PeligroCO; 
  jsonRoot["LPG"]   = peligro_aire.PeligroLPG; 
  jsonRoot["Humo"]  = peligro_aire.PeligroHumo; 
  
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
  
  
  while (WiFi.status() != WL_CONNECTED) { // Una vez conectados, sale del bucle while y prosigue con el código
    delay(500);
    Serial.print(".");
  }
  
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  Serial.printf("Connected, mac address: %02X:%02X:%02X:%02X:%02X:%02X\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]); // MAC del dispositivo
  
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  

}
//----------------------- FIN CONEXIÓN A INTERNET ----------------------- 

//-----------------------[ CONEXIÓN MQTT ]-----------------------
//Bucle de conexion MQTT

void reconnect() {
  struct registro_conexion estado;          // Declaramos la estructura para conexión (json)
  
  // Permanecemos en el bucle hasta que nos reconectemos
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Se crea un cliente el ID propio de cada chip
    String clientId = "ESP8266Client-";
    clientId += String(ESP.getChipId()) ;
    
    // Esperamos a conexión
    estado_conex    = false;
    estado.conexion = estado_conex;                  // Inicialmente, no hemos establecido conexión
   
    // Conexión al servidor y últimas voluntades
    if (client.connect(clientId.c_str(),"infind","zancudo",topic_conexion,0,true,serializa_JSON(estado).c_str())) {
      Serial.println("connected"); // Estaremos conectados. Usuario: infind. Contraseña: zancudo. 
      estado.CHIPI=String(ESP.getChipId());
      estado_conex    = true;
      estado.conexion = estado_conex;
      
      client.publish(topic_conexion,serializa_JSON(estado).c_str(),true );
      
      // Subscripción a los distintos topics
      client.subscribe(topic_FOTA);         // Subscripción al topic de comprobación de actualizaciones
      client.subscribe(topic_led_cmd);      // Subscripción al topic del estado del GPIO 2
      client.subscribe(topic_switch_cmd);   // Subscripción al topic del estado del GPIO 16
      client.subscribe(topic_config);       // Subscripción al topic de configuración
      client.subscribe(topic_tipo_logica);  // Subscripicón al topic del tipo de lógica (positiva o negativa)
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

  // Comprobación ------Actualizaciones------
  if(strcmp(topic,topic_tipo_logica)==0) // Se estudia el valor del topic de Actualización
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
        bool logica = root["logica"];            // Buscamos actualizaciones
        logica_PN=logica;
        Serial.println(logica);
      }
      free(mensaje); // libero memoria
   }
  
  // Comprobación ------Actualizaciones------
  if(strcmp(topic,topic_FOTA)==0) // Se estudia el valor del topic de Actualización
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
        actualiza = root["actualiza"];            // Buscamos actualizaciones
        origen_act="mqtt";                        // El origen de búsqueda de actualizaciones se debe a un mensaje recibido por MQTT
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
      
      if (switch_max == 0)                
      {
        estado_led16 = 0;
        if (logica_PN==false)             // Switch apagado - Log Neg
        {
          
          digitalWrite(LED_Secundario,HIGH);
        }
        else                              // Switch encendido - Log Pos
        {
          digitalWrite(LED_Secundario,LOW);
        }
      }
      else if (switch_max == 1)           
      {
        estado_led16 = 1;
        if (logica_PN==false)             // Switch encendido - Log Neg
        {
          
          digitalWrite(LED_Secundario,LOW);
        }
        else                              // Switch apagado - Log Pos
        {
          digitalWrite(LED_Secundario,HIGH);
        }
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
    else                                // En caso de no haber error, se hace la secuencia propuesta:
    {
     LED_max = root["level"];           // Se toma el valor del dato de entrada
     origen_LED="mqtt";
     cambia_PWM=true;
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
        float valorDatos     = root["envia"];      // Se toman los valores corresponientes al tiempo del envío de mensajes (s)
        float valorActualiza = root["actualiza"];  // Se toma el tiempo de comprobación de actualizaciones (min)
        float valorVelocidad = root["velocidad"];  // Se toma la velocidad de cambio de salida PWM (ms)
        float valorLED       = root["LED"];        // Configura si salida está activada/encendida a nivel alto(1) o bajo(0). null: se mantiene configuración
        int valorSWITCH      = root["SWITCH"];     // Mismas condiciones que para valorLED

        // -- Envia -- 
        if (root["envia"] == nullptr)              // Valor recibido sin valor, mantenemos el último valor
        {
        }
        else                                       
        {
          TiempoRecogida=valorDatos;
        }
        // -- Actualiza -- 
        if (root["actualiza"] == nullptr)              // Valor recibido sin valor, mantenemos el último valor
        {
        }
        else
        {
          TiempoActualiza=valorActualiza;          // Asignamos a una variable global ya que se usará en otros bucles externos al 'Callback'
        }

        // -- Velocidad -- 
        if (root["velocidad"] == nullptr)              // Valor recibido sin valor, mantenemos el último valor
        {
        }
        else
        {
          TiempoLED=valorVelocidad;
        }
        // -- LED -- 
        if (root["LED"] == nullptr)                 // Valor recibido sin valor, mantenemos el último valor
        {
        }
        else if (valorLED == 0)                     // Valor recibido = 0, apagamos LED
        {
          if(logica_PN==false)
          {
            // Apagamos LED - LOG NEG
            LED_max = 0;
          }
          else
          {
            // Encendemos LED - LOG POS
            LED_max = 100;
          }
          cambia_PWM = true;
          origen_LED = "mqtt";
        }
        else if (valorLED == 1)                  // Valor recibido = 1, encendemos al valor máximo el LED
        {
          if(logica_PN==false)
          {
            // Encendemos LED - LOG NEG
            LED_max = 100;
          }
          else
          {
            // Apagamos LED - LOG POS
            LED_max = 0;
          }
          cambia_PWM = true;
          origen_LED = "mqtt";
        }
        
        // -- SWITCH --
        if (root["SWITCH"] == nullptr)              // Valor recibido sin valor, mantenemos el último valor
        {
          // Mantiene valor anterior
        }
        else if (valorSWITCH == 0)                  // Valor recibido = 0, apagamos SWITCH
        {
          if (logica_PN==false)
            digitalWrite(LED_Secundario,HIGH);      // Apagamos SWITCH - LOG NEG
          else
            digitalWrite(LED_Secundario,LOW);       // Encendemos SWITCH - LOG POS
          
          estado_led16 = 0;
          
          // Publicamos por el topic correspondiente
          valor_SWITCH.SWITCH = estado_led16;
          valor_SWITCH.origen = "mqtt";
          //Publicamos los datos
          client.publish(topic_switch,serializa_JSONSWITCH(valor_SWITCH).c_str());  //Publico que he recibido el dato del switch
        }
        else if (valorSWITCH == 1)                  // Valor recibido = 1, encendemos SWITCH
        {
          if (logica_PN==false)
            digitalWrite(LED_Secundario,LOW);       // Encendemos SWITCH - LOG NEG
          else
            digitalWrite(LED_Secundario,HIGH);      // Apagamos SWITCH - LOG POS
          
          estado_led16=1; 
          
          // Publicamos por el topic correspondiente
          valor_SWITCH.SWITCH = estado_led16;
          valor_SWITCH.origen = "mqtt";
          //Publicamos los datos
          client.publish(topic_switch,serializa_JSONSWITCH(valor_SWITCH).c_str());  //Publico que he recibido el dato del switch
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

  
  // MQTT y CALLBACK
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);

  // Al iniciar, ocurre la actualización
  actualiza=true;
  origen_act="inicio";

  // Buffer
  client.setBufferSize(512);              //Ampliamos el tamaño del buffer
  char cadena[512];

  // FLASH
   pinMode(boton_flash, INPUT_PULLUP);    // Inicializamos el pin LED_Secundario como entrada
   
  // Activamos la interrupción
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);
 
  // Asignaciones para uso de sensores, en caso de obtener otro ID, se le asignará como habitación
  if (String(ESP.getChipId())=="5821487")         // Sensores disponibles: DHT11 / VL53L0X / MQ2
    nombre_casa="salon";
  else if (String(ESP.getChipId())=="1122489")    // Sensores disponibles: DHT11
    nombre_casa="wc";
  else if (String(ESP.getChipId())=="5826449")    // Sensores disponibles: DHT11 / MQ2
    nombre_casa="cocina";
  else if (String(ESP.getChipId())=="8789990")    // Sensores disponibles: DHT11
    nombre_casa="dormitorio";
  else
    nombre_casa="habitacion";                     // Sensores disponibles: DHT11

  

  // Iniciamos sensores correspondientes
  if((nombre_casa=="salon")or(nombre_casa=="wc")or(nombre_casa=="cocina")or(nombre_casa=="dormitorio")or(nombre_casa=="habitacion"))
  { 
    // DHT
    dht.setup(14, DHTesp::DHT11);           // Connect DHT sensor to GPIO 14
  }
  if ((nombre_casa=="salon")or(nombre_casa=="cocina"))
  {
    // MQ-2
    mq2.begin();
  }
  if (nombre_casa=="salon")
  {
    // LÁSER
    comprueba_laser=true;
  }
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
  
   // ------CONEXIÓN-----
  String mensaje_conexion;
  if (!client.connected()) {
    reconnect();
  }
    
  client.loop();
  
  // Declaramos inicialmente estructuras json que se emplearán
  struct registro_conexion estado;          // Declaramos la estructura para conexión (json)
  struct registro_datos datos_globales;     // json para datos globales
  struct registro_led valor_LED;            // Declaramos registro json para valores del LED
  struct registro_distancia laser;          // json para sensor laser
  struct registro_MQ2 peligro_aire;         // json para sensor de Peligro de aire 
  struct registro_actualizacion estado_act; // json para confirmar búsqueda de actualizaciones

  float PWM;
  unsigned long now=millis();
  
    // -----INTERRUPCIONES-----
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
      cont++;                           // Se ha dado un pulso 
      if (ahora-penultima_int > 2000)   // Si NO haya actualización, se estudian las pulsaciones
      {
        // Si se mantiene +2seg pulsado, se habilita la señal que permite actualizar
        actualiza=true;
        cont=0;                         // Para evitar encender o apagar el LED, se resetea el contador de pulsos
        
        // Hacemos saber que la actualización se debe por el pulsador flash
        origen_act="pulsador";
      }
    }
    interrupcion=false;                     // Desabilitamos la interrupción hasta que vuelva a haber una llamada a la misma
  }
    now=millis();

    // -----ESTUDIO PULSACIONES-----
      if (now-inicio>650)                   // Cuando pasen 0.65 seg tras la interrupción
      {
        if (cont==2)                        // Si se han contado 2 pulsos -> encendemos al nivel máximo el GPIO 16 (Log neg) o apagamos al mínimo en Log pos
        {
          if (logica_PN==false) {
          Serial.println("DOBLE PULSACION");
          Serial.println("GPIO 2 ENCENDIDO al MÁX");
          }
          else
            Serial.println("GPIO 2 APAGADO al MÍN");

          // Activamos la regulación por PWM
          LED_max=100;
          cambia_PWM=true;
          cont=0;                             // Reseteamos la cuenta de pulsos
          cuenta_pulsos=true;                 // Se activa la contación de pulsos a partir del nº de interrupciones
        }
        else if (cont==1)                     // Si solo se ha contado 1 pulsación, encendemos o apagamos el LED principal (GPIO 16)
        {
        Serial.println("UNA PULSACION");
        // Con una pulsación, se apaga en caso de estar encendido o bien se enciende al nivel anterior en caso de estar apagado
        if (estado_led2==1)
          {
            // En caso de estar encendido, se apaga
            if (logica_PN==false)
              LED_max=0;
            else
              LED_max=100;
              
            Serial.println("GPIO 2 APAGADO");
          }
          else                                // Si estaba apagado, encendemos al nivel anterior
          { 
            LED_max=LED_anterior;
            Serial.println("GPIO 2 ENCENDIDO AL NIVEL ANTERIOR");
          }
          // Activamos la regulación por PWM
          cambia_PWM=true;
          cont=0;                             // Reseteamos la cuenta
          cuenta_pulsos=true;
          origen_LED="pulsador";
        }
        else if (cont>2)                      // Si se cuentan más de 2 pulsos, se hace saber que no es válido
        {
          Serial.println("MAS DE 2 PULSACIONES, CÓDIGO NO VÁLIDO");
          cont=0;
          cuenta_pulsos=true;
        }
        
    delay(10);
    }

    // -----PWM-----
    // Únicamente entra en este condicionante cuando se le llame por MQTT o se pulse el botón flash (GPIO0)
    if(cambia_PWM==true) {
      float subirled=LED_dato;
      // Aumentamos intensidad del LED
      while(subirled<LED_max)            // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
      {
        // Se actualiza la intensidad del LED a través de PWM 
        if (logica_PN==false)             // Lógica negativa: subirled=0 -> PWM=1023 // subirled=100 -> PWM=0
          PWM = 1023*(1-subirled/100);    
        else                              // Lógica positiva: subirled=0 -> PWM=0    // subirled=100 -> PWM=1023
          PWM = 1023*(subirled/100);
        
        analogWrite(BUILTIN_LED,PWM);     //Envio el valor al pin del led
     
        // Publicamos por el topic correspondiente
        valor_LED.LED    = subirled;
        valor_LED.origen = origen_LED; 
        client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() ); 
     
        subirled++;
        delay(TiempoLED);                 // Se espera el tiempo exigido por el usuario
      }

      // Disminuimos intensidad del LED
      while(subirled>LED_max)            // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
      {
      if (logica_PN==false)              // Lógica negativa: subirled=0 -> PWM=1023 // subirled=100 -> PWM=0
        PWM = 1023*(1-subirled/100);    
      else                              // Lógica positiva: subirled=0 -> PWM=0    // subirled=100 -> PWM=1023
        PWM = 1023*(subirled/100);
        
      analogWrite(BUILTIN_LED,PWM); 
     
      // Publicamos por el topic correspondiente
      valor_LED.LED    = subirled;
      valor_LED.origen = origen_LED; 
      client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() ); 
     
      subirled--;
      delay(TiempoLED);
      }
      if (logica_PN==false)             // Lógica negativa: subirled=0 -> PWM=1023 // subirled=100 -> PWM=0
        PWM = 1023*(1-subirled/100);    
      else                              // Lógica positiva: subirled=0 -> PWM=0    // subirled=100 -> PWM=1023
        PWM = 1023*(subirled/100);
        
      analogWrite(BUILTIN_LED,PWM);      //Envio el valor al pin del led
     
      LED_dato = subirled;

      // Para poder encender al nivel anterior a través del botón flash, se usa esta variable intermedia 
      if (logica_PN==false)
      {
        if (LED_dato != 0)              // Evita seguir apagado en caso de que el valor previo sea 0 -Lóg Neg
          LED_anterior = LED_dato; 
      }
      else                             // Evita seguir apagado en caso de que el valor previo sea 100 -Lóg Pos
      {
        if (LED_dato != 100)
          LED_anterior = LED_dato;
      }
      
      // Publicamos por el topic correspondiente
      valor_LED.LED    = LED_dato;
      valor_LED.origen = origen_LED; 
      client.publish(topic_led,serializa_JSONLED(valor_LED).c_str() );

      // Estudio del estado del LED (apagado o encendido)
      if (logica_PN==false)
      {
        if(LED_dato==0)       // LED apagado   - Lóg Neg
          estado_led2=0;
        else                  // LED encendido - Lóg Neg
          estado_led2=1;
      }
      else
      {
        if(LED_dato==100)     // LED apagado   - Lóg Pos
          estado_led2=0;
        else                  // LED encendido - Lóg Pos
          estado_led2=1;
      }
      cambia_PWM=false;
    }


    now=millis();
  
    // -----DATOS GLOBALES-----
    if (now - lastMsg > TiempoRecogida*1000) {   // [segundos] -> [ms]
  
      //Las variables que vamos a meter en nuestra estructura datos
      datos_globales.ssid         = WiFi.localIP().toString();
      datos_globales.mqtt_server  = WiFi.SSID();
      datos_globales.vcc          = ESP.getVcc();
      datos_globales.rssi         = WiFi.RSSI();
      datos_globales.temperatura  = dht.getTemperature();
      datos_globales.humedad      = dht.getHumidity();
      datos_globales.tiempo       = millis();
      datos_globales.valor_LED    = LED_dato;
      datos_globales.valor_SWITCH = estado_led16;
      datos_globales.CHIPI        = String(ESP.getChipId());  
   
      client.publish(topic_global,serializa_JSONGlobal(datos_globales).c_str() );
      //Publicamos el estado de conexion con retain flag=true

      estado.conexion=estado_conex;
      client.publish(topic_conexion,serializa_JSON(estado).c_str(),true );
      
      lastMsg = now;
    }
  
  
    now=millis();
    // -----ACTUALIZACIÓN-----
    if (now - lastAct > TiempoActualiza*60000 && TiempoActualiza != 0)   // [min -> segundos]
    {
      actualiza=true;                           // En caso de que se cumpla el Tiempo deseado para buscar actualizaciones, se habilita la señal actualiza, tomando el valor true

      // Hacemos saber que la actualización se debe por vencimiento de tiempo
      origen_act="mqtt";
    }
  
    if (actualiza==true )                       // Mantenemos pulsado +2s o vence el tiempo de actualización: ACTUALIZAMOS! 
    {
      // OTA
      Serial.println( "--------------------------" );
      Serial.println( "Comprobando actualización:" );
      Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH); // Se comprueba la actualización
      Serial.println( "--------------------------" );  
      
      actualiza=false;                        // Se 'resetea' la variable booleana que habilita la actualización
      
      // Hacemos saber al usuario que se van a buscar actualizaciones
      estado_act.actualizacion=false;
      estado_act.origen=origen_act;
      client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str());
      
      ESPhttpUpdate.setLedPin(16,LOW);
      ESPhttpUpdate.onStart(inicio_OTA);
      ESPhttpUpdate.onError(error_OTA);
      ESPhttpUpdate.onProgress(progreso_OTA);
      ESPhttpUpdate.onEnd(final_OTA);
      
      switch(ESPhttpUpdate.update(OTA_URL, HTTP_OTA_VERSION, OTAfingerprint)) {
        case HTTP_UPDATE_FAILED:
          Serial.printf(" HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println(F(" El dispositivo ya está actualizado"));
          
          // Hacemos saber al usuario que el dispositivo se ha actualizado
          estado_act.actualizacion=true;
          estado_act.origen=origen_act;
          client.publish(topic_actualizacion,serializa_JSONact(estado_act).c_str());
        break;
        case HTTP_UPDATE_OK:
          Serial.println(F(" OK"));
        break;
      }
      lastAct = now; 

      
    }
    now=millis();
    // -----LASER-----
    if (nombre_casa=="salon") {
      if ((comprueba_laser == true)&&(now-lastLaser>20000))   // Se reintentará cada cierto tiempo la inicialización del sensor láser
      {
        if (!lox.begin()) {                                   // En caso de fallo de inicialización, se toma el valor -1 en estado_laser
          Serial.println(F("Error al iniciar VL53L0X"));
          laser.estado_sensor = "Fallo al iniciar";           // Se hace saber que ha dado fallo al inicializar
          estado_laser = -1;
        }
        else                                                  // En caso de haberse inicializado correctamente, se toma un valor alto (7000) en estado_laser
        {
          Serial.println("Iniciado VL53L0X");
          laser.estado_sensor = "VL53L0X activo";             // Se hace saber que el sensor se ha inicializado correctamente
          estado_laser=7000;
        
          comprueba_laser = false;                            // No volvemos a estudiar la inicialización, sino que se empiezan a tomar las medidas correspondientes
        }
        // Publicamos el estado por el topic correspondiente
        laser.distancia = estado_laser;
        client.publish(topic_laser,serializa_JSONdistist(laser).c_str() ); 

        lastLaser=now;
      }
      else if(comprueba_laser==false)                         // Cuando esté activo el sensor, se podrá leer datos del sensor
      {
        VL53L0X_RangingMeasurementData_t measure;
  
        // Leyendo sensor...
        lox.rangingTest(&measure, false);                     // Si se pasa true como parametro, muestra por puerto serie datos de debug
        float medida_laser = measure.RangeMilliMeter;

        laser.distancia     = medida_laser;
        laser.estado_sensor = "VL53L0X iniciado";

        if (medida_laser<=200 && cambia_dist==0)              // Envío de datos únicamente en caso de estar por debajo del umbral (puerta abierta) y la última fue por encima
        {
          Serial.print("Distancia (mm): ");
          Serial.println(measure.RangeMilliMeter);
    
          // Publicamos la distancia por el topic correspondiente
          client.publish(topic_laser,serializa_JSONdistist(laser).c_str() ); 
          
          cambia_dist=1;
        }
        else if (medida_laser>=200 && cambia_dist==1)         // Envío de datos únicamente en caso de estar por encima del umbral (puerta cerrada) y la última fue por debajo
        {
          Serial.print("Distancia (mm): ");
          Serial.println(measure.RangeMilliMeter);
    
          // Publicamos la distancia por el topic correspondiente
          client.publish(topic_laser,serializa_JSONdistist(laser).c_str() ); 
          cambia_dist=0;
        }
      }
    }
    
    
    // -----PELIGROSIDAD MQ2-----
    // Se analizan los niveles y se determina la peligrosidad respectiva
    if ((nombre_casa=="salon")or(nombre_casa=="wc")or(nombre_casa=="cocina")) {
      // Se analizan los niveles de CO 
      if (mq2.readCO()>=1.2)
      {
        PeligroCOa=3;
      }
      else if (mq2.readCO()>=0.9 && mq2.readCO()<1)
      {
        PeligroCOa=2;
      }
      else if (mq2.readCO()>=0.8 && mq2.readCO()<0.9)
      {
        PeligroCOa=1;
      }
      else
      {
        PeligroCOa=0;
      }

      //Se analizan los niveles de LPG
      if (mq2.readLPG()>=1.2)
      {
        PeligroLPGa=3;
      }
      else if (mq2.readLPG()>=0.9 && mq2.readLPG()<1)
      {
        PeligroLPGa=2;
      }
      else if (mq2.readLPG()>=0.8 && mq2.readLPG()<0.9)
      {
        PeligroLPGa=1;
      }
      else
      {
        PeligroLPGa=0;
      }

      // Se analizan los niveles de LPG
      if (mq2.readSmoke()>=1.2)
      {
        PeligroHumoa=3;
      }
      else if (mq2.readSmoke()>=0.9 && mq2.readSmoke()<1)
      {
        PeligroHumoa=2;
      }
      else if (mq2.readSmoke()>=0.8 && mq2.readSmoke()<0.9)
      {
        PeligroHumoa=1;
      }
      else
      {
        PeligroHumoa=0;
      }
      now=millis();
    
      // En caso de haber haber peligro en algún valor, se hace saber al usuario
      if ((PeligroHumoa>0 || PeligroLPGa>0 || PeligroCOa>0)||(now-lastPeligro>20000))
      {
        //Valores de Peligrosidad
        peligro_aire.PeligroCO   = PeligroCOa; 
        peligro_aire.PeligroLPG  = PeligroLPGa; 
        peligro_aire.PeligroHumo = PeligroHumoa; 
  
        client.publish(topic_mq2,serializa_JSONMQ2(peligro_aire).c_str() );
        //Publicamos el estado de conexion con retain flag=true
        lastPeligro=now;
      }
    }
    delay(10);
}
//----------------------- FIN LOOP ----------------------- 
