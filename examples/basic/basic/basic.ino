#include <Arduino.h>
#include <LIS2MDL_Driver.h>


#define SDA_PIN 32
#define SCL_PIN 33

#define SET 36

#define AVG_LENGTH 10
float average_buf[AVG_LENGTH];
int8_t avg_index = 0;
float average(float input);

LIS2MDL compass;

float average(float input)
{

  avg_index++;
  avg_index = avg_index % AVG_LENGTH;
  average_buf[avg_index] = input;

  float min = 360;
  float max = 0;
  float sum = 0;
  for (float val : average_buf)
  {
    if (val < min)
      min = val;
    if (val > max)
      max = val;
    sum += val;
  }

  // DEBUG_PRINTF("Min: %f  Max: %f\n", min, max);
  return sum / AVG_LENGTH;
}

void setup() {
  pinMode(SET, INPUT);
  Serial.begin(9600);
  Serial.println();
  Serial.println("Starting");
  Wire.begin(SDA_PIN, SCL_PIN);

  compass.begin();
  compass.setODR(ODR::ODR_100_Hz);
  int16_t Xofst = -69;
  int16_t Yofst = -438;
  compass.writeXOffset(Xofst);
  compass.writeYOffset(Yofst);
  compass.enableAvgFilter();
  compass.filter_alpha = 0.05;
}
float X, Y, Z;
void loop() {

  if (!digitalRead(SET))
    compass.enableAvgFilter(!compass.getAvgFilter());
    while(!digitalRead(SET)){};
    //compass.calibrate();
  Serial.print(0);
  Serial.print('\t');
  Serial.print(0);
  Serial.print('\t');
  Serial.print(compass.getHeading());
  Serial.print('\t');
  
  Serial.println(average(round(compass.getHeading())));
  delay(10);
}


