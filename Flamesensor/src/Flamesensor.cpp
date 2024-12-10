


#include "Particle.h"
 int flameSensorPin1 =A0;
 int flameSensorPin2 =A1;
 int value;
 int heat;
SYSTEM_MODE(AUTOMATIC);


SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);


void setup() {
Serial.begin(9600);
waitFor(Serial.isConnected,10000);


}


void loop() {
  value=analogRead(flameSensorPin1);
  heat=analogRead(flameSensorPin2);
  Serial.printf("value=%i,heat=%i,\n",value,heat);


  
}
