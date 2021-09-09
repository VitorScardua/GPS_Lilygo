/*****************************************
* ESP32 GPS VKEL 9600 Bds
******************************************/

#include <TinyGPS++.h>  

TinyGPSPlus gps;

double alt;
String read_sentence;                     

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 34, 12);   //34-TX 12-RX, refenrete a pinagem da placa Lilygo
}


void loop()
{
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 6); //Variável com valor de latitude,  imprimi no terminal com 6 digitos.
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 6); //Variável com valor de longitude,  imprimi no terminal com 6 digitos.
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value()); //Variável com valor de satélites conectados.
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808); //Variável com valor de altitude.
  alt = gps.altitude.feet() / 3.2808; //Salva o valor de altitute na variavel "alt" para envio de criação de pacote.
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour()); //Variável de hora.
  Serial.print(":");
  Serial.print(gps.time.minute()); //Variável de minuto.
  Serial.print(":");
  Serial.println(gps.time.second()); //Variável de segundo.
  Serial.println("**********************");

  gps_payload(gps.location.lat(),gps.location.lng(), alt); //Envia os valores do GPS (Lat, Long, Alt) para a criação do payload.

  smartDelay(1000);                                      
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring")); //Caso não encontre antena.
}


/*
  Função de Delay
*/
static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

/*
  Função para a criação do Payload e imprimi o payload em Hexa no monitor serial.
*/
void gps_payload(float lat, float lon, int alt) {

  uint32_t LatitudeBinary = 0;
  uint32_t LongitudeBinary = 0;

  // Caso o valor de lat não seja zero, faz a multiplicação dos valores para que não sejam enviados com ",".
  if (lat != 0){
    LatitudeBinary = (lat  * 10000);
    LongitudeBinary = (lon  * 10000);
    alt = alt * 100;
  }
  
  //Inicializa e cria o payload
  uint8_t payload[10];

  payload[0] = LatitudeBinary >> 24 ;
  payload[1] = LatitudeBinary >> 16;
  payload[2] = LatitudeBinary >> 8;
  payload[3] = LatitudeBinary;

  payload[4] = LongitudeBinary >> 24;
  payload[5] = LongitudeBinary >> 16;
  payload[6] = LongitudeBinary >> 8;
  payload[7] = LongitudeBinary;

  payload[8] = alt >> 8;
  payload[9] = alt;

  // Imprime os valores de Payload (neste caso são bytes)
  // Os 8 primeiros bits serão lat, seguido de 8 de long e por fim 4 de alt.
  int x = 0;
  for (x=0; x <=9; x++) {
    Serial.print(payload[x], HEX);
  }
  Serial.println(" ");
}
