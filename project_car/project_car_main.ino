
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
const float Kp = 0.035; // determine experimentally
const float Kd = 0.09;
const int LEFT_THRESHOLD = 450;
const int RIGHT_THRESHOLD = -400;

// min and max from calibration
const int sensorMins[8] = {596,596,596,619,666,573,642,619};
const int sensorMax[8] = {1849,1894,1846,1620,1834,1582,1858,1881};
const int calibrationWeights [2][8] = {{-8,-6,-4,-8,8,15,24,28},{-28,-24,-15,-8,8,4,6,8}}; // go forward, go backward
const int start_speed = 35;


// define variables + arrays
int sensorCalc[8] ={0};
int calibrationWeightUsed[8];
int thisCalWeight = 0;
int PDval = 0;

uint16_t sensor_measured[8] = {0};
int calcError = 0;
int prevError = 0;

// 14.6 cm in diameter
int readSum = 0;
int hasTurned = 0; // true if travelling down, false if travelling up

// for split
int right_baseSpeed = 25;
int left_baseSpeed = 25;

// define pins
const int left_nslp_pin =31;     // nslp ==> awake & ready for PWM
const int right_nslp_pin=11;     // nslp ==> awake & ready for PWM
const int left_dir_pin  =29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 41; // blinks when 180
const int LED_LF = 51; // blinks when split detected

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
    digitalWrite(left_dir_pin,LOW);    // LOW vs. HIGH changes direction (HIGH = C, LOW = CC)
    digitalWrite(left_nslp_pin,HIGH);   // make car vroom
    digitalWrite(right_dir_pin,LOW);
    digitalWrite(right_nslp_pin,HIGH);

    pinMode(LED_RF, OUTPUT);
    pinMode(LED_LF, OUTPUT);

    // initialize serial communication
    ECE3_Init();
    Serial.begin(9600);  // set the data rate in bits per second for serial data transmission

}

// LOOP (program run continiously as car is on)
void loop(){
    // read sensor Values
    digitalWrite(LED_RF, LOW);
    digitalWrite(LED_LF, LOW);
    ECE3_read_IR(sensor_measured);

    // compute error
    calcError = 0; //VS
    // subtract mins
    for(unsigned char i = 0; i < 8; i++){
        sensorCalc[i] = sensor_measured[i] - sensorMins[i];
    }
    // normalize to 1000
    for(unsigned char j = 0; j < 8; j++){
        sensorCalc[j] = sensorCalc[j]*1000/sensorMax[j];
    }

    // add bias
    if(hasTurned == 1)
        thisCalWeight = 1; //RIGHT BIAS
    else
        thisCalWeight = 0; //LEFT BIAS

    // calculate error
    for(unsigned char k = 0; k < 8; k++){
        calcError += sensorCalc[k] * calibrationWeights[thisCalWeight][k];
    }
    calcError /= 8;

    // check for crosspiece
    readSum = 0;
    for(int i = 0; i < 8 ; i++){
        readSum += sensorCalc[i];
    }
    readSum /= 8;
    if (readSum > 900){
        hasTurned += 1;
        switch(hasTurned) {
            case 1:
                // turn 180
                digitalWrite(LED_LF, HIGH); 
                analogWrite(left_pwm_pin, 50);
                analogWrite(right_pwm_pin, 50);
                digitalWrite(left_dir_pin,LOW);     
                digitalWrite(right_dir_pin,HIGH);
                delay(1290);

                // set back to forwards
                digitalWrite(right_dir_pin,LOW);
                break;
            case 2:
                digitalWrite(LED_LF, HIGH);
                break;
            case 3:
                digitalWrite(LED_LF, HIGH);
                digitalWrite(LED_RF, HIGH);
                analogWrite(left_pwm_pin,0);
                analogWrite(right_pwm_pin, 0);
                digitalWrite(left_nslp_pin,LOW);
                digitalWrite(right_nslp_pin,LOW);
                return;
        }
    }

    // calculate PDval
    PDval = calcError * Kp + (calcError - prevError) * Kd;

    // add change to one wheel, subtract from other
    left_baseSpeed = start_speed - PDval;
    right_baseSpeed = start_speed + PDval;
    // adjust the steering
    if (calcError > LEFT_THRESHOLD) { // left wheel slower
        //TURN LEFT
        digitalWrite(left_dir_pin,HIGH); 
        digitalWrite(right_dir_pin,LOW); 
        if(left_baseSpeed < 0){
            left_baseSpeed *= -1;
        }
    } else if(calcError < RIGHT_THRESHOLD) { // right wheel slower
        //TURN RIGHT
        digitalWrite(left_dir_pin,LOW); 
        digitalWrite(right_dir_pin,HIGH); 
        if(right_baseSpeed < 0){
            right_baseSpeed *= -1;
        }
    } else {
        //GO FORWARD 
        digitalWrite(left_dir_pin,LOW); 
        digitalWrite(right_dir_pin,LOW); 
    }

    prevError = calcError;

    analogWrite(left_pwm_pin,left_baseSpeed);
    analogWrite(right_pwm_pin, right_baseSpeed);

}