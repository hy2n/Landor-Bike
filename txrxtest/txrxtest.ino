#include <SoftwareSerial.h>
 
SoftwareSerial gpsSerial(A12,A13);
 
void setup() {
  Serial.begin(115200);
  Serial.println("Start GPS... ");
  gpsSerial.begin(9600);
}
 
void loop() {
  if(gpsSerial.available())
  {
    Serial.write(gpsSerial.read());
  }
}