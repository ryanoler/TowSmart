#include "Particle.h"
#include "math.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"
#include "Grove_Air_Quality_Sensor.h"

byte accel_x_h, accel_x_l;
byte accel_y_h, accel_y_l;
byte accel_z_h, accel_z_l;
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
float xGravity, yGravity, zGravity;
float pitch, roll, pitchDeg, rollDeg;
float totalAccel; // Total acceleration (magnitude)
 int flameSensorPin1 =A0;
 int flameSensorPin2 =A1;
 int MPU_ADDRESS=0x68;
 int Value;
unsigned int last, lastTime;
int pubValue;
AirQualitySensor sensor(A2);

TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
 // pitchDeg=360/(2*M_PI)*pitch;rollDeg=360/(2*M_PI)*roll;
Adafruit_MQTT_Publish pitchFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pitchDeg");
Adafruit_MQTT_Publish rollFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rollDeg");
Adafruit_MQTT_Publish totalAccelFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/totalAccel");



void MQTT_connect();
bool MQTT_ping();
SYSTEM_MODE(AUTOMATIC);

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);


void setup()
{
  WiFi.on();
  WiFi.connect();
  while (WiFi.connecting()){
     Serial.printf(".");
  }
  Serial.printf("\n\nConnected to Wi-Fi\n");
  // pubFeed.publish()
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
}

void loop() {
  
 MQTT_connect();
 MQTT_ping();
    // x
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 2, true);
  accel_x_h = Wire.read();
  accel_x_l = Wire.read();
  accel_x = accel_x_h << 8 | accel_x_l;
  xGravity = (1 / 16384.0) * accel_x;
  Serial.printf("X-axis acceleration is %i \n", accel_x);
 
  Serial.printf("xGravity=%0.2f\n", xGravity);
    // y
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 2, true);
  accel_y_h = Wire.read();
  accel_y_l = Wire.read();
  accel_y = accel_y_h << 8 | accel_y_l;
  Serial.printf("Y-axis acceleration is %i \n", accel_y);
    yGravity = (1 / 16384.0) * accel_y;
    Serial.printf("yGravity=%0.2f\n", yGravity);
    // z
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3f);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 2, true);
    accel_z_h = Wire.read();
    accel_z_l = Wire.read();
    accel_z = accel_z_h << 8 | accel_z_l;
    Serial.printf("Z-axis acceleration is %i \n", accel_z);
    zGravity = (1 / 16384.0) * accel_z;
    Serial.printf("zGravity=%0.2f\n", zGravity);

    pitch = -asin(xGravity);
    roll = atan2(yGravity, zGravity);
    pitchDeg = 360 / (2 * M_PI) * pitch;
    rollDeg = 360 / (2 * M_PI) * roll;
    totalAccel = sqrt((xGravity * xGravity) + (yGravity * yGravity) + (zGravity * zGravity));
    Serial.printf("Total Acceleration = %.2f g\n", totalAccel);
  
     // Publish data to MQTT server
    if((millis()-lastTime>6000)){
    if (mqtt.Update()) {
      pitchFeed.publish(pitchDeg);   // Publish pitch degree to MQTT
      rollFeed.publish(rollDeg);     // Publish roll degree to MQTT
      totalAccelFeed.publish(totalAccel);  // Publish total acceleration to MQTT
    }  
      lastTime =millis();
      
    
    }
}
   void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>6000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;

}
//unsigned long previousMillis = 0; // stores the last time the sensor data was updated
//const long interval = 10000;

//void MQTT_connect(){
  //if(!mqtt.connected()){
    //Serial.printf("Connect to MQTT...");
    //while (!mqtt.connect()){
      //Serial.print(".");
      //delay(500);
    //}
    //Serial.printf("Connected to MQTT.");
  //}
//}


   //MQTT_connect();
  //MQTT_ping();
  //unsigned long currentMillis = millis(); // Get the current time

  // Check if 10 seconds have passed
  //if (currentMillis - previousMillis >= interval){
  
    // Save the last time we updated the data
   //previousMillis = currentMillis;

   //byte axes [3] ={0x3B,0x3D,0x3F};
    //int16_t accel[3];

   //for(int i= 0; i< 3; i++){
    //Wire.beginTransmission(MPU_ADDRESS);
    //Wire.write(axes[i]);
    //Wire.endTransmission(false);
    //Wire.requestFrom(MPU_ADDRESS,2, true);
    //byte highByte = Wire.read();
    //byte lowByte =Wire.read();
    //accel[i]=(highByte <<8) | lowByte;
    //Serial.printf("%c-axis acceleration is %i\n", 'X', accel[i]);

   //}
    //xGravity =(1/16384.0)*accel[0];
    //yGravity =(1/16384.0)*accel[1];
    //zGravity =(1/16384.0)*accel[2];
    //Serial.printf("%c-axis acceleration is %i\n",'x'+i,);  
  


