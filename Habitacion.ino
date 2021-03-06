#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino_JSON.h>


// DHT
#include "DHTesp.h"
ADC_MODE(ADC_VCC);
DHTesp dht;

// Datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("192.168.1.90")        // Dirección del servidor de actualización OTA 192.168.1.113
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
  int SWITCH;
  char CHIPI;
  };
  
// Estructura de json para publicar conexión
  struct registro_conexion { //estructur  a para el estado de la conexion
    char CHIPI;
    bool conexion;
  };
  
  // Estructura de json para publicar actualización
  struct registro_actualizacion { //estructura para el estado de la actualización
    bool actualizacion;
  };
  

// Estructura de json para publicar valor del LED
  struct registro_led {
    char CHIPI;
    float LED;
    String origen;
  };
// Estructura de json para publicar valor del SWITCH
  struct registro_switch {
    char CHIPI;
    int SWITCH;
    String origen;
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

const char* ssid = "iPhone de Frank";
const char* password = "Sarumordor21";
//const char* ssid = "vodafoneAAR6W7"; // JAVI: ALFIL_PISOS // LEO: vodafoneAAR6W7 // 
//const char* password = "Hn3fF6xXYFbgPJsH"; // JAVI: alfil2020 // LEO: Hn3fF6xXYFbgPJsH
const char* mqtt_server = "iot.ac.uma.es"; // Servidor MQTT común a todos los integrantes del grupo


WiFiClient espClient;

PubSubClient client(espClient);

// Se define el ancho del buffer para las estructuras json
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg_led[MSG_BUFFER_SIZE];      // Para valor de LED
char msg_recDatos[MSG_BUFFER_SIZE]; // Para recogida de datos
char msg_switch[MSG_BUFFER_SIZE];   // Para valor de SWITCH




// OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);
unsigned long lastAct = 0; // última 

// Interrupciones
unsigned long lastMsg = 0; // último mensaje para datos enviado
bool actualiza = false; 
bool cuenta_pulsos=true;
bool pulsaciones=false;

unsigned long inicio = 0; 
int cont=0;

int LED_Secundario = 16; //GPIO 16 - LED de ABAJO
//int BUILTIN_LED = 2; // GPIO 0 - LED de ARRIBA
volatile int estado_led2=1; // Inicialmente, ambos LEDs están encendidos: estado=1. En caso de estar apagados, estado=0
volatile int estado_led16=1;

//PWM
int value = 0;
double LED_dato = 100; // tomamos que inicialmente está encendida al 100%
volatile double LED_max=100;
volatile double PWM;
double LED_anterior = LED_dato;

//Recogida de datos
double TiempoRecogida=10000; // Recoger datos sensores
double TiempoLED=10; // Tiempo en que tarda en cambiar el LED un 1%
int TiempoActualiza = 60; // viene dado en minutos


//-----------------------CONVERSIÓN A STRING----ESTRUCTURAS JSON-----------------------
//Esta funcion nos convierte a un String todos los datos globales para ser interpretados por Json
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
  
  jsonRoot["CHIPID"]=datos.CHIPI; // ID del Chip
  jsonRoot["Uptime"]= datos.tiempo;
  jsonRoot["vcc"] = datos.vcc/1000.;
  jsonRoot["DHT11"]= DHT11;
  jsonRoot["Wifi"]=Wifi;
  jsonRoot["LED"]=datos.LED;
  jsonRoot["SWITCH"]=datos.SWITCH;


  return JSON.stringify(jsonRoot);
}
// -----------------------
// Función para la conexion
String serializa_JSON (struct registro_conexion estado)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["CHIPID:"] = estado.CHIPI;
  jsonRoot["online:"] = estado.conexion;
  
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

// Función para el valor del LED
String serializa_JSONLED (struct registro_led valor_LED)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["CHIPID"]=valor_LED.CHIPI; // ID del Chip
  jsonRoot["led"]=valor_LED.LED; // ID del Chip
  jsonRoot["origen"]=valor_LED.origen; // ID del Chip
  
  return JSON.stringify(jsonRoot);
}
// -----------------------

// Función para el valor del SWITCH
String serializa_JSONSWITCH (struct registro_switch valor_SWITCH)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["CHIPID"]=valor_SWITCH.CHIPI; // ID del Chip
  jsonRoot["switch"]=valor_SWITCH.SWITCH; // ID del Chip
  jsonRoot["origen"]=valor_SWITCH.origen; // ID del Chip
  
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
    
    if (client.connect(clientId.c_str(),"infind","zancudo","infind/GRUPO2/ESP230/conexion/habitacion",0,true,serializa_JSON(estado).c_str())) {
      Serial.println("connected"); // Estaremos conectados. Usuario: infind. Contraseña: zancudo. 
      estado.CHIPI=ESP.getChipId();
      estado.conexion = true;
      //"{\"online\":false}"
     client.publish("infind/GRUPO2/ESP230/conexion/habitacion",serializa_JSON(estado).c_str(),true); // Publicamos por el topic correspondiente el estado de conexión
     
     client.subscribe("infind/GRUPO2/ESP230/led/cmd/habitacion"); //Me suscribo al topic del estado del GPIO 2
     client.subscribe("infind/GRUPO2/ESP230/switch/cmd/habitacion"); //Me suscribo al topic del estado del GPIO 16

     client.subscribe("infind/GRUPO2/ESP230/config/cmd/habitacion"); //Me suscribo al topic del estado del led
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
  
  struct registro_led valor_LED;        // Declaramos registro json para valores del LED
  struct registro_switch valor_SWITCH;  // Declaramos registro json para valores del SWITCH
  
  // compruebo que es el topic adecuado ------SWITCH------
  if(strcmp(topic,"infind/GRUPO2/ESP230/switch/cmd/habitacion")==0)
  {
    StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje
    // Deserializa el documento JSON
    DeserializationError error = deserializeJson(root, mensaje);

    // Compruebo si hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else  // En caso de no haber error, se hace la secuencia propuesta:
    {
      double switch_max = root["level"]; // Se toma el valor del dato de entrada
      
      if (switch_max == 0)      // Switch apagado
      {
        estado_led16 = 0;
        digitalWrite(LED_Secundario,HIGH);
      }
      else if (switch_max == 1) // Switch encendido
      {
        estado_led16 = 1;
        digitalWrite(LED_Secundario,LOW);
      }

      // Publicamos por el topic correspondiente
      valor_SWITCH.CHIPI=ESP.getChipId();
      valor_SWITCH.SWITCH=estado_led16;
      valor_SWITCH.origen="mqtt"; 
      client.publish("infind/GRUPO2/ESP230/switch/status/habitacion",serializa_JSONSWITCH(valor_SWITCH).c_str()); //Publico que he recibido el dato del switch
    }
  }
  
  // compruebo que es el topic adecuado ------LED-----
  if(strcmp(topic,"infind/GRUPO2/ESP230/led/cmd/habitacion")==0)
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
     LED_max = root["level"]; // Se toma el valor del dato de entrada
     
     double subirled=LED_dato; // Iniciamos variable
     
     // Aumentamos intensidad del LED
     while(subirled<LED_max) // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
     {
     PWM = 1023*(1-subirled/100);  // Se actualiza la intensidad del LED a través de PWM
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
     // Publicamos por el topic correspondiente
     valor_LED.CHIPI=ESP.getChipId();
     valor_LED.LED=subirled;
     valor_LED.origen="mqtt"; 
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/ESP230/led/status/habitacion",serializa_JSONLED(valor_LED).c_str() ); //Publico que he recibido el dato del led
     subirled++;
     delay(TiempoLED); // Se espera el tiempo exigido por el usuario
     }

     // Disminuimos intensidad del LED
     while(subirled>LED_max) // Mientras no se alcance el valor exigido (en caso de tener valor del led por debajo del exigido), se repite el bucle 
     {
     PWM = 1023*(1-subirled/100); 
     analogWrite(BUILTIN_LED,PWM); 
     
     // Publicamos por el topic correspondiente
     valor_LED.CHIPI=ESP.getChipId();
     valor_LED.LED=subirled;
     valor_LED.origen="mqtt"; 
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/ESP230/led/status/habitacion",serializa_JSONLED(valor_LED).c_str() ); //Publico que he recibido el dato del led
     
     subirled--;
     delay(TiempoLED);
     }
     
     PWM = 1023*(1-LED_max/100); //Finalmente, se alcanza el valor de LED propuesto por el usuario
     analogWrite(BUILTIN_LED,PWM); //Envio el valor al pin del led
     
     LED_dato = subirled;
     if (LED_dato != 0)
      LED_anterior = LED_dato;
     // Publicamos por el topic correspondiente
     valor_LED.CHIPI=ESP.getChipId();
     valor_LED.LED=LED_dato;
     valor_LED.origen="mqtt"; 
     //publica que he llegado a ese valor
     client.publish("infind/GRUPO2/ESP230/led/status/habitacion",serializa_JSONLED(valor_LED).c_str() ); //Publico que he recibido el dato del led
     
    
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
  free(mensaje); // libero memoria
  }
  }

    //------RECOGIDA DATOS------
    if(strcmp(topic,"infind/GRUPO2/ESP230/config/cmd/habitacion")==0) // Se estudia el estado del topic específico
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
        double valorDatos = root["envia"];          // Se toman los valores corresponientes al tiempo del envío de mensajes (s)
        double valorActualiza = root["actualiza"];  // Se toma el tiempo de comprobación de actualizaciones (min)
        double valorVelocidad = root["velocidad"];  // Se toma la velocidad de cambio de salida PWM (ms)
        float valorLED = root["LED"];               // Configura si salida está activada/encendida a nivel alto(1) o bajo(0). null: se mantiene configuración
        int valorSWITCH = root["SWITCH"];           // Mismas condiciones que para valorLED

        TiempoActualiza=valorActualiza; // Asignamos a una variable global ya que se usará en otros bucles externos al 'Callback'
        TiempoRecogida=valorDatos;
        TiempoLED=valorVelocidad;
        // -- LED -- 
        if (root["LED"] == nullptr) // == nullptr
        {
          Serial.println("Valor LED NULL");
          analogWrite(BUILTIN_LED,LED_max);
        }
        else if (valorLED == 0)
        {
          digitalWrite(LED_Secundario,HIGH);
          estado_led2 = 0;
          LED_dato = 0;
        }
        else if (valorLED == 1)
        {
          digitalWrite(LED_Secundario,LOW);
          estado_led2=1;
          LED_dato = 100;
        }
        // -- SWITCH --
        if (root["SWITCH"] == nullptr)
        {
          // Mantiene valor anterior
        }
        else if (valorSWITCH == 0)
        {
          digitalWrite(LED_Secundario,HIGH);
          estado_led16 = 0;
        }
        else if (valorSWITCH == 1)
        {
          digitalWrite(LED_Secundario,LOW);
          estado_led16=1;
        }
          

        free(mensaje); // libero memoria
      }
  }
}

//-----------------------SET-UP-----------------------
void setup() {
  struct registro_actualizacion estado_act; // Declaramos la estructura para actualización (json)
  pinMode(BUILTIN_LED, OUTPUT);    // Inicializamos el pin BUILTIN_LED como salida
  pinMode(LED_Secundario, OUTPUT); // Inicializamos el pin LED_Secundario como salida
  digitalWrite(LED_Secundario, LOW);
  digitalWrite(BUILTIN_LED, LOW);
  Serial.begin(9600);
  setup_wifi();
  // OTA -> Actualizamos al empezar
      client.subscribe("infind/GRUPO2/ESP230/FOTA/habitacion"); //Me suscribo al topic de la actualización
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
   pinMode(boton_flash, INPUT_PULLUP); // Inicializamos el pin LED_Secundario como entrada
  // Activamos la interrupción
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);
  

  
    //MQ-2
//    mq2.begin();
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
  struct registro_led valor_LED;        // Declaramos registro json para valores del LED
  if (interrupcion==true) // En caso de que se active la interrupción
  {
    if (cuenta_pulsos==true) 
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
    
    unsigned long now=millis();
    
      if (now-inicio>650) // Cuando pasen 0.65 seg tras la interrupción
      {
        if (cont==2) // Si se han contado 2 pulsos -> encendemos o apagamos el LED principal (GPIO2)
        {
          Serial.println("DOBLE PULSACION");
          digitalWrite(BUILTIN_LED,LOW);     // Encendemos al nivel máximo
          Serial.println("GPIO 2 ENCENDIDO al MÁX");
          estado_led2=1;
          LED_dato = 100;
          LED_anterior = LED_dato;
          
          cont=0; // Reseteamos la cuenta
          cuenta_pulsos=true;

          // Publicamos por el topic correspondiente
          valor_LED.CHIPI=ESP.getChipId();
          valor_LED.LED=LED_dato;
          valor_LED.origen="pulsador"; 
          //publica que he llegado a ese valor
          client.publish("infind/GRUPO2/ESP230/led/status/habitacion",serializa_JSONLED(valor_LED).c_str() ); //Publico que he recibido el dato del led
        }
        if (cont==1) // Si solo se ha contado 1 pulsación, encendemos o apagamos el LED secundario (GPIO 16)
        {
        Serial.println("UNA PULSACION");

        if (estado_led2==1) // Si estaba encendido, apagamos
          {
            digitalWrite(BUILTIN_LED,HIGH);
            Serial.println("GPIO 2 APAGADO");
            estado_led2 = 0;
            LED_dato = 0;
            
          }
          else              // Si estaba apagado, encendemos al nivel anterior
          { 
            PWM = 1023*(1-LED_anterior/100); 
            analogWrite(BUILTIN_LED,PWM);
            Serial.println("GPIO 2 ENCENDIDO AL NIVEL ANTERIOR");
            estado_led2 = 1;
            LED_dato = LED_anterior;
            
          }
          cont=0; // Reseteamos la cuenta
          cuenta_pulsos=true;
          
          // Publicamos por el topic correspondiente
          valor_LED.CHIPI=ESP.getChipId();
          valor_LED.LED=LED_dato;
          valor_LED.origen="pulsador"; 
          client.publish("infind/GRUPO2/ESP230/led/status/habitacion",serializa_JSONLED(valor_LED).c_str()); //Publico que he recibido el dato del switch
        }
        else if (cont>2) // Si se cuentan más de 2 pulsos, se hace saber que no es válido
        {
          Serial.println("MAS DE 2 PULSACIONES, CÓDIGO NO VÁLIDO");

          cont=0;
          cuenta_pulsos=true;
        }
    delay(10);
    }
  

  // CONEXIÓN
  String mensaje_conexion;
  if (!client.connected()) {
    reconnect();
  }
    
  client.loop();
  //unsigned long now = millis();


  // DATOS
  struct registro_datos misdatos; //me creo una estructura de cada tipo
  
  
  if (now - lastMsg > TiempoRecogida) {
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
   misdatos.SWITCH = estado_led16;
   // ID del CHIP
   misdatos.CHIPI = ESP.getChipId();
   
// Publicamos los datos por el topic correspondiente
  client.publish("infind/GRUPO2/ESP230/datos/habitacion",serializa_JSON2(misdatos).c_str() );
//Publicamos el estado de conexion con retain flag=true

    lastMsg = now; // Actualizamos el tiempo en que se produce la publicación de datos
  }

  if (now - lastAct > TiempoActualiza*60000 || TiempoActualiza != 0)
  {
    actualiza==true;
  }
  
  if (actualiza==true) // mantenemos pulsado: ACTUALIZAMOS! 
    {
      client.subscribe("infind/GRUPO2/ESP230/FOTA/habitacion"); //Me suscribo al topic de la actualización cuando se mantenga más de 2 segundos el botón flash pulsado
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
      actualiza=false;
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
      lastAct = now; 
    }

  delay(10);
}
