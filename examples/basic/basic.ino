#include <LIS2MDL.h>


#define SDA_PIN 32
#define SCL_PIN 33

#define SET 36

LIS2MDL compass;

void setup() {
  pinMode(SET, INPUT);
  Serial.begin(9600);
  Serial.println();
  Serial.println("Starting");
  Wire.begin(SDA_PIN, SCL_PIN);

  compass.begin();
  compass.setODR(ODR::ODR_100_Hz);


}
float X, Y, Z;
void loop() {
  if (!digitalRead(SET))
    compass.calibrate();
  
  Serial.println(compass.getHeading());
  delay(10);
}
