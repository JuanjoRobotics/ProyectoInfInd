// Estructura de json para publicar valor de Peligrosidad
  struct registro_Peligro {
    char CHIPI;
    int PeligroCO;
    int PeligroLPG;
    int PeligroHumo;
  };

//Peligro
int PeligroCOa=0; //Nivel de peligro de CO
int PeligroLPGa=0; //Nivel de peligro de LPG
int PeligroHumoa=0; //Nivel de peligro de Humo

// Función para el valor de la Peligrosidad
String serializa_JSONPeligro (struct registro_Peligro valor_Peligro)
{
  JSONVar jsonRoot;
  JSONVar ANALOG;
  String jsonString;
  

  jsonRoot["CO"]=valor_Peligro.PeligroCO; // 
  jsonRoot["LPG"]=valor_Peligro.PeligroLPG; // 
  jsonRoot["Humo"]=valor_Peligro.PeligroHumo; // 
  
  return JSON.stringify(jsonRoot);
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

    //Analizamos los niveles de CO 
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

    //Analizamos los niveles de LPG
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

    //Analizamos los niveles de LPG
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

    if ( PeligroHumoa>0 || PeligroLPGa>0 || PeligroCOa>0)
    {
  // DATOS
  struct registro_Peligro NivelPeligro; //me creo una estructura de cada tipo

  //Valores de Peligrosidad
  NivelPeligro.PeligroCO=PeligroCOa; // 
  NivelPeligro.PeligroLPG=PeligroLPGa; //
  NivelPeligro.PeligroHumo=PeligroHumoa; //
  
  // Publicamos los datos por el topic correspondiente
  client.publish("infind/GRUPO2/ESP145/Peligro/cocina",serializa_JSONPeligro(NivelPeligro).c_str() );
  //Publicamos el estado de conexion con retain flag=true
    }
    
}
