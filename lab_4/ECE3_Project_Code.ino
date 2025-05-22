#include <ECE3.h>

#include <ECE3.h>
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;
const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;
const int LED_RF = 41;


const int LED_RF = 41;
uint16_t sensorValues[8];
int trackRating = 0;
int current_position = -40;
int increment_position = 4;
int trackRatingNew = 0;
int number_samples = 5;

void setup()
{
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
  
}


void loop()
{
  // Reset varfiables
  int summed_values[8] = {0};
  int leftSpd = 20;
  int rightSpd = 20;
  
  trackRating = 0;
  // Take the average of 5 consecutive values for each sensor
  for (int j = 0; j < number_samples; j++){
    // Read raw sensor values
    ECE3_read_IR(sensorValues);

    // Add the current sensor values using a for loop
    for (unsigned char i = 0; i < 8; i++)
    {
      summed_values[i] += sensorValues[i];
      switch (i)
      {
        case 0:
        {
          trackRating -= 16*1000*(sensorValues[i]-731)/1769;
          break;
        }
        case 1:
        {
          trackRating -= 8*1000*(sensorValues[i]-792)/1708;
          break;
        }
        case 2:
        {
          trackRating -= 4*1000*(sensorValues[i]-731)/1769;
          break;
        }
        case 3:
        {
          trackRating -= 2*1000*(sensorValues[i]-685)/1815;
          break;
        }
        case 4:
        {
          trackRating += 2*1000*(sensorValues[i]-685)/1815;
          break;
        }
        case 5:
        {
          trackRating += 4*1000*(sensorValues[i]-731)/1769;
          break;
        }
        case 6:
        {
          trackRating += 8*1000*(sensorValues[i]-754)/1746;
          break;
        }
        case 7:
        {
          trackRating += 16*1000*(sensorValues[i]-778)/1722;
          break;
        }
      }
    }
  }
  int speed = 20 - 20*trackRating/86000;
  double kp = 0.5;
  double kd = 0.125;
  double trackRatingDeriv =  127*(trackRating*127/86000+0.6*127*log(2.72*trackRating/86000) - trackRatingNew)/8600;
  double trackRatingNew = 0.4*trackRating*127/86000+0.6*127*log(2.72*trackRating/86000);

  leftSpd = speed - (kp*trackRatingNew+kd*trackRatingDeriv);
  rightSpd = speed + (kp*trackRatingNew+kd*trackRatingDeriv);
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);
  delay(4);
  }
