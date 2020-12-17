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

int salida = 2; //GPIO 2
int estado_led=0;
bool cambia=false;
/*------------------ SETUP --------------------*/
void setup() {
  pinMode(boton_flash, INPUT_PULLUP);
  // descomentar para activar interrupción
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Interrupción preparada...");
  pinMode(salida, OUTPUT); 

}

/*------------------- LOOP --------------------*/
void loop() {
  
  if (interrupcion==true)
  {      
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
      cambia=true;
      if (estado_led==1)
        estado_led=0;
      else
        estado_led=1;
    } 
    interrupcion=false; // DESABILITAMOS LA INTERRUPCIÓN HASTA QUE VUELVA A HABER UNA LLAMADA A LA MISMA
    }
    if (cambia==true)
    {
      if (estado_led==1)
      {
        digitalWrite(salida, HIGH);
      }
      else
      { 
        digitalWrite(salida, LOW); 
      }
        cambia=false;
    }
    delay(10);
    
  }
  
