
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
const float Kp = 0.05; // determine experimentally
const float Kd = 0.09;
const int errorMax = 2777;

// min and max from calibration
const int sensorMins[8] = {596,596,596,619,666,573,642,619};
const int sensorMax[8] = {1849,1894,1846,1620,1834,1582,1858,1881};
const int calibrationWeights [2][8] = {{-8,-6,-4,-8,8,15,24,28},{-28,-24,-15,-8,8,4,6,8}}; // go forward, go backward
const int start_speed = 50;

const int mult_zero[8] = {0,0,1,1,1,1,0,0};

// define variables + arrays
int sensorCalc[8] ={0};
int calibrationWeightUsed[8];
int centerSum = 0;

int num_peak = 0;
int thisCalWeight = 0;

uint16_t sensor_measured[8] = {0};
int calcError = 0;
int prevError = 0;

// 14.6 cm in diameter
int readSum = 0;
int hasTurned = 0; // true if travelling down, false if travelling up
int turnTime = 0;

// for split

float last_time = 0;
float now_time = 0;
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

     
    //delay(2000);

}

// LOOP (program run continiously as car is on)
void loop(){
    //analogWrite(left_pwm_pin,left_baseSpeed);
    //analogWrite(right_pwm_pin, right_baseSpeed);

    // read sensor Values
    digitalWrite(LED_RF, LOW);
    digitalWrite(LED_LF, LOW);
    ECE3_read_IR(sensor_measured);

    if(hasTurned == 3){
        digitalWrite(LED_LF, HIGH);
        analogWrite(left_pwm_pin,0);
        analogWrite(right_pwm_pin, 0);
        digitalWrite(left_nslp_pin,LOW);
        digitalWrite(left_nslp_pin,LOW);
        return;
    }
        // compute error
    compute_error(sensor_measured); // changes calcError and isCrosspiece

    // check for crosspiece
    readSum = 0;
    for(int i = 0; i < 8 ; i++){
        readSum += sensorCalc[i];
    }
    readSum /= 8;
    if (readSum > 900){ //CHANGED THRESHOLD VS
        hasTurned += 1;
        switch(hasTurned) {
            case 1:
                digitalWrite(LED_LF, HIGH); 
                rotate180();
                break;
            case 2:
                digitalWrite(LED_LF, HIGH);
                break;
            case 3:
                digitalWrite(LED_LF, HIGH);
                analogWrite(left_pwm_pin,0);
                analogWrite(right_pwm_pin, 0);
                digitalWrite(left_nslp_pin,LOW);
                digitalWrite(left_nslp_pin,LOW);
        }
    }


    int PDval = getPD();

    // add change to one wheel, subtract from other

    left_baseSpeed = start_speed - PDval;
    right_baseSpeed = start_speed + PDval;
    adjust_steer();            // adjust the steering

    prevError = calcError;
    analogWrite(left_pwm_pin,left_baseSpeed);
    analogWrite(right_pwm_pin, right_baseSpeed);

}

// HELPER FUNCTIONS

// computes the error of array of 8 sensor inputs by fusing each value according to values in calibrationWeight
void compute_error (uint16_t sensorValues[8]){
    calcError = 0; //VS
    // subtract mins
    for(unsigned char i = 0; i < 8; i++){
        sensorCalc[i] = sensorValues[i] - sensorMins[i];
    }
    // normalize to 1000
    for(unsigned char j = 0; j < 8; j++){
        sensorCalc[j] = sensorCalc[j]*1000/sensorMax[j];
    }

    // add bias
    if(hasTurned)
        thisCalWeight = 1; //RIGHT BIAS
    else
        thisCalWeight = 0; //LEFT BIAS

    // if centered ignore other values
    for(unsigned char k = 0; k < 8; k++){
        calibrationWeightUsed[k] = calibrationWeights[thisCalWeight][k]; 
    }

    // calculate error
    for(unsigned char k = 0; k < 8; k++){
        calcError += sensorCalc[k] * calibrationWeightUsed[k];
    }
    calcError /= 8;
    return;
}
// computes steering change according to error determined by weights on track
void adjust_steer(){
    if (calcError > 450) { // left wheel slower
        //TURN LEFT
        digitalWrite(left_dir_pin,HIGH); 
        digitalWrite(right_dir_pin,LOW); 
        left_baseSpeed -= 10;
        right_baseSpeed -= 10;
        if(left_baseSpeed < 0){
            left_baseSpeed *= -1;
        }
    } else if(calcError < -400) { // right wheel slower
        //TURN RIGHT
        digitalWrite(left_dir_pin,LOW); 
        digitalWrite(right_dir_pin,HIGH); 
        right_baseSpeed -= 10;
        left_baseSpeed -= 10;
        if(right_baseSpeed < 0){
            right_baseSpeed *= -1;
        }
    } else {
        //GO FORWARD 
        right_baseSpeed = start_speed; //VS
        left_baseSpeed = start_speed;
        digitalWrite(left_dir_pin,LOW); 
        digitalWrite(right_dir_pin,LOW); 
    }
    return;
}

// rotate 180
void rotate180(){
    analogWrite(left_pwm_pin, 50);
    analogWrite(right_pwm_pin, 50);
    digitalWrite(left_dir_pin,LOW);    
    digitalWrite(left_nslp_pin,HIGH);   
    digitalWrite(right_dir_pin,HIGH);
    digitalWrite(right_nslp_pin,HIGH);

    delay(1290); //VS og value 1310
    analogWrite(left_pwm_pin, 50);
    analogWrite(right_pwm_pin, 50);
    digitalWrite(left_dir_pin,LOW);    
    digitalWrite(left_nslp_pin,HIGH);   
    digitalWrite(right_dir_pin,LOW);
    digitalWrite(right_nslp_pin,HIGH);
    return;
}

// calc initial PD
int getPD(){
    return calcError * Kp + (calcError - prevError) * Kd;
}