// project_car_main.
// 
// * 
// *    
// *    
// *    
// *  Names: Clara Yee + Vaish Sam
// *  UID (Clara): 406392066
// *  UID (Vaish):

#include <ECE3.h> // Used for encoder functionality

// DEFINE INITIAL VALUES
// define constants

// min and max from calibration
const int sensorMins[8] = {436,527,505,482,550,459,574,527};
const int sensorMax[8] = {1895,1729,1490,877,1635,1606,1926,1973};
const int calibrationWeight[8] = {-8,-4,-2,-1,1,2,4,8};

// define variables + arrays
int sensorCalc[8] ={0};
int calcError = 0;
uint16_t sensor_measured[8] = {0};

int baseSpeed = 25;

// define pins
const int left_nslp_pin =31;     // nslp ==> awake & ready for PWM
const int right_nslp_pin=11;     // nslp ==> awake & ready for PWM
const int left_dir_pin  =29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

// SETUP (program run once for initialization)
void setup(){
    // set pin modes 
    pinMode(left_nslp_pin,OUTPUT);
    pinMode(left_dir_pin,OUTPUT);
    pinMode(left_pwm_pin,OUTPUT);
    pinMode(right_nslp_pin,OUTPUT);
    pinMode(right_dir_pin,OUTPUT);
    pinMode(right_pwm_pin,OUTPUT);

    // initialize pins
    // default is forward
    digitalWrite(left_dir_pin,HIGH);    // LOW vs. HIGH changes direction (HIGH = C, LOW = CC)
    digitalWrite(left_nslp_pin,HIGH);   // make car vroom
    digitalWrite(right_dir_pin,HIGH);
    digitalWrite(right_nslp_pin,HIGH);

    // initialize serial communication
    ECE3_Init();
    Serial.begin(9600);  // set the data rate in bits per second for serial data transmission

    // set base speed
    analogWrite(left_pwm_pin,baseSpeed);
    analogWrite(right_pwm_pin, baseSpeed);
}

// LOOP (program run continiously as car is on)
void loop(){
    // read sensor Values
    ECE3_read_IR(sensor_measured);

    // check for crosspiece

    // compute error
    compute_error(sensor_measured);
    // compute steering change command
    // add change to one wheel, subtract from other
}

// HELPER FUNCTIONS

// computes the error term by fusing each value according to values in calibrationWeight
void compute_error (int sens[8]){
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
    return;
}

// computes steering change according to error determined by weights on track
void adjust_steer(){}

// turns right
void turn_right(){
    digitalWrite(left_dir_pin,HIGH); 
    digitalWrite(right_dir_pin,LOW); 
}
// turns left
void turn_left(){
    digitalWrite(left_dir_pin,LOW); 
    digitalWrite(right_dir_pin,HIGH); 
}