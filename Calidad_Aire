const int MQ_PIN = 2;
const int MQ_DELAY = 2000;
 
void setup()
{
  Serial.begin(9600);
}
 
 
void loop()
{
  bool state= digitalRead(MQ_PIN);
 
  if (!state)
  {
    Serial.println("Deteccion");
  }
  else
  {
    Serial.println("No detectado");
  }
  delay(MQ_DELAY);
}
