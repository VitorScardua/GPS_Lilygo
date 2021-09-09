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
  Serial1.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX
}


void loop()
{
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 6);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  alt = gps.altitude.feet() / 3.2808;
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");

  gps_payload(gps.location.lat(),gps.location.lng(), alt);

  smartDelay(1000);                                      
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


void gps_payload(float lat, float lon, int alt) {

  uint32_t LatitudeBinary = 0;
  uint32_t LongitudeBinary = 0;

  if (lat != 0){
    LatitudeBinary = (lat  * 10000);
    LongitudeBinary = (lon  * 10000);
    alt = alt * 100;
    Serial.println(lat, 6);
    //Serial.println(LatitudeBinary);
    Serial.println(lon, 6);
    //Serial.println(LongitudeBinary);
    Serial.println(alt);
  }
  
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
  
  //Serial.print(payload[0], HEX);
  Serial.print(payload[1], HEX);
  Serial.print(payload[2], HEX);
  Serial.println(payload[3], HEX);

  Serial.print(payload[4], HEX);
  Serial.print(payload[5], HEX);
  Serial.print(payload[6], HEX);
  Serial.println(payload[7], HEX);

  int x = 0;
  for (x=0; x <=9; x++) {
    Serial.print(payload[x], HEX);
  }
  Serial.println(" ");
}
