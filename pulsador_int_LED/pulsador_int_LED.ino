// -------------------------------------------------------------------
// Ejemplo de contol de un pulsador mediante polling e interrupciones
// Alumno: LEONARDO GARCÍA GUILLÉN
// -------------------------------------------------------------------

int boton_flash=0;       // GPIO0 = boton flash
int estado_polling=HIGH; // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
volatile int estado_int=HIGH;     // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW

unsigned long inicio_pulso_polling = 0;

volatile unsigned long ahora; // SE PONDRÁ GLOBAL PARA EN LOOP() CONOCER EL VALOR ACTUAL QUE SE TOMA EN LA INTERRUPCIÓN
volatile unsigned long penultima_int = 0; // CREO ESTA VARIABLE PARA ENVIAR EL ULTIMO_DATO Y USARLO EN EL INT ANTES DE ACUTALIZARSE AL DATO ACTUAL
unsigned long ultima_int = 0;

volatile bool interrupcion=false; // SE USARÁ ESTA VARIABLE PARA INDICAR CUÁNDO SE DA UNA INTERRUPCIÓN

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

int salida = 16; //GPIO 16 - LED de ABAJO
//int BUILTIN_LED = 0 // GPIO 0 - LED de ARRIBA
int estado_led0=1;
int estado_led16=1;


/*------------------ SETUP --------------------*/
void setup() {
  pinMode(boton_flash, INPUT_PULLUP); 

  // descomentar para activar interrupción
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);
  Serial.begin(9600);
  Serial.println();
  Serial.println("Interrupción preparada...");
  pinMode(salida, OUTPUT); // GPIO 16
  pinMode(BUILTIN_LED, OUTPUT); // GPIO 0

}
int cont=0;
bool cuenta_pulsos=true;
bool actualiza=false;
unsigned long inicio = 0; 
bool pulsaciones=false;
/*------------------- LOOP --------------------*/
void loop() {
  
  // INTERRUPCIÓN
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
      if (ahora-penultima_int < 3000) // Mientras NO haya actualización
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
        Serial.println("ACTUALIZACIÓN");
        actualiza=false;
    }
    
    unsigned long now=millis();
   
      if (now-inicio>650 && cont==2) // 2 pulsos - encendemos arriba
      {
        Serial.println("DOBLE PULSACION");
        if (estado_led16==1)
          {
            digitalWrite(BUILTIN_LED, HIGH);
            estado_led16=0;
          }
          else
          { 
            digitalWrite(BUILTIN_LED, LOW);
            estado_led16=1;
          }
          cont=0;
          cuenta_pulsos=true;
      }
      else if (now-inicio>650 && cont==1)
      {
        Serial.println("UNA PULSACION");
        if (estado_led0==1)
          {
            digitalWrite(salida, HIGH);
            estado_led0=0;
          }
          else
          { 
            digitalWrite(salida, LOW);
            estado_led0=1;
          }
          cont=0;
          cuenta_pulsos=true;
      }
    delay(10);
    }
  
  
