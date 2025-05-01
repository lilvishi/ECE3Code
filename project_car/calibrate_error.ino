#include <ECE3.h>


uint16_t sensorValues[8];
// from calibration
const int sensorMins[8] = {436,527,505,482,550,459,574,527};
const int sensorMax[8] = {1895,1729,1490,877,1635,1606,1926,1973};
const int calibrationWeight[8] = {-8,-4,-2,-1,1,2,4,8};

// va
int sensorCalc[8] ={0};
int calcError = 0;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
}


void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

    // subtract mins
  for(unsigned char i = 0; i < 8; i++){
        sensorCalc[i] = sensorValues[i] - sensorMins[i];
  }

    // normalize to 1000
    for(unsigned char j = 0; j < 8; j++){
        sensorCalc[j] = sensorCalc[j]*1000/sensorMax[j];
  }

  // calculate error
    for(unsigned char k = 0; k < 8; k++){
        calcError += sensorCalc[k] * calibrationWeight[k];
  }
  calcError /= 4;

  Serial.print(calcError);
  

  delay(50);
}