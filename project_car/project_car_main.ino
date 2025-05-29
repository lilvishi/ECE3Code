
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
const float Kp = 0.03; // determine experimentally
const float Kd = 0.003;
const int errorMax = 2777;

// min and max from calibration
const int sensorMins[8] = {436,527,505,482,550,459,574,527};
const int sensorMax[8] = {1895,1729,1490,877,1635,1606,1926,1973};
const int calibrationWeights [3][8] = {{-8,-4,-2,-1,1,2,4,8},{-6,-2,-1,-1,2,3,6,12},{-12,-6,-3,-2,1,1,2,6}}; // no change, go forward, go backward
const int start_speed = 25;

const int mult_zero[8] = {0,0,1,1,1,1,0,0};

// define variables + arrays
int sensorCalc[8] ={0};
int calibrationWeightUsed[8];
int centerSum = 0;

int num_peak = 0;
int thisCalWeight = 0;

uint16_t sensor_measured[8] = {0};
float calcError = 0;
float prevError = 0;
float dt = 0;

// 14.6 cm in diameter
int readSum = 0;
bool hasTurned = false; // true if travelling down, false if travelling up
float turnTime = 0;

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

    // set base speed
    analogWrite(left_pwm_pin,left_baseSpeed);
    analogWrite(right_pwm_pin, right_baseSpeed);
    delay(2000);

}

// LOOP (program run continiously as car is on)
void loop(){
    now_time = millis();
    if(now_time < turnTime){
        rotate180();
        digitalWrite(LED_RF, HIGH);
    }
    else if (now_time == turnTime){
        forward();
    }
    else{
        // read sensor Values
        digitalWrite(LED_RF, LOW);
        ECE3_read_IR(sensor_measured);

        // compute error
        compute_error(sensor_measured); // changes calcError and isCrosspiece

        // check for crosspiece
        readSum = 0;
        for(int i = 0; i < 8 ; i++){
            readSum += sensorCalc[i];
        }
        readSum /= 8;
        if (readSum > 900){
            turnTime = now_time + 1310;
            hasTurned = !hasTurned;
        }

        // compute steering change command
        dt = now_time - last_time;
        last_time = now_time;

        int PDval = getPD();

        // add change to one wheel, subtract from other

        left_baseSpeed = start_speed - PDval;
        right_baseSpeed = start_speed + PDval;
        adjust_steer();            // adjust the steering

        prevError = calcError;
        analogWrite(left_pwm_pin,left_baseSpeed);
        analogWrite(right_pwm_pin, right_baseSpeed);
        }
}

// HELPER FUNCTIONS

// computes the error of array of 8 sensor inputs by fusing each value according to values in calibrationWeight
void compute_error (uint16_t sensorValues[8]){
    // subtract mins
    for(unsigned char i = 0; i < 8; i++){
        sensorCalc[i] = sensorValues[i] - sensorMins[i];
    }
    // normalize to 1000
    for(unsigned char j = 0; j < 8; j++){
        sensorCalc[j] = sensorCalc[j]*1000/sensorMax[j];
    }

    // check if there is a split 
    num_peak = 0;
    thisCalWeight = 0;

    for(unsigned char k = 0; k < 8; k++){
        if(sensorCalc[k] > 900)
            num_peak +=1;
    }

    // check if the center is reading
    centerSum = 0;
    centerSum += sensorCalc[3];
    centerSum += sensorCalc[4];

    // if centered ignore other values
    for(unsigned char k = 0; k < 8; k++){
        // if there are two peaks at once choose is arch or split
        if(num_peak == 2){
            digitalWrite(LED_LF, HIGH);

            // add bias
            if(hasTurned)
                thisCalWeight = 2;
            else
                thisCalWeight = 1;

            // if arch ignore side, otherwise keep bias
            //if(is arch)
                //calibrationWeightUsed[k] = calibrationWeights[thisCalWeight][k] * mult_zero[k]; 
            //else 
            calibrationWeightUsed[k] = calibrationWeights[thisCalWeight][k]; 
        }
        else{ // check for peaks if not centered
            digitalWrite(LED_LF, LOW);
            calibrationWeightUsed[k] = calibrationWeights[thisCalWeight][k]; 
        }
    }

    // calculate error
    for(unsigned char k = 0; k < 8; k++){
        calcError += sensorCalc[k] * calibrationWeightUsed[k];
    }
    calcError /= 4;
    return;
}
// computes steering change according to error determined by weights on track
void adjust_steer(){
    if (calcError > 200) { // left wheel slower
        turn_left();
        }
    else if(calcError < -200) { // right wheel slower
        turn_right();
        }
    else{
        forward();
        }
    return;
}

// turns right
void turn_right(){
    digitalWrite(left_dir_pin,LOW); 
    digitalWrite(right_dir_pin,HIGH); 
    right_baseSpeed -= 10;
    left_baseSpeed -= 10;
    if(right_baseSpeed < 0){
        right_baseSpeed *= -1;
        //right_baseSpeed += 10;
    }
    //right_baseSpeed /= 2;
}
// turns left
void turn_left(){
    digitalWrite(left_dir_pin,HIGH); 
    digitalWrite(right_dir_pin,LOW); 
    right_baseSpeed -= 10;
    left_baseSpeed -= 10;
    if(left_baseSpeed < 0){
        left_baseSpeed *= -1;
        //left_baseSpeed += 10;
    }
    //left_baseSpeed /= 2;
}
// goes forward
void forward(){
    digitalWrite(left_dir_pin,LOW); 
    digitalWrite(right_dir_pin,LOW); 
}

// rotate 180
void rotate180(){
    analogWrite(left_pwm_pin, 50);
    analogWrite(right_pwm_pin, 50);
    digitalWrite(left_dir_pin,LOW);    
    digitalWrite(left_nslp_pin,HIGH);   
    digitalWrite(right_dir_pin,HIGH);
    digitalWrite(right_nslp_pin,HIGH);
}

// calc initial PD
int getPD(){
    return calcError * Kp + ((calcError - prevError)/dt) * Kd;
}