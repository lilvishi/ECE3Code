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
const int baseSpeed = 25;

// define variables + arrays
const int base_speed[8] = [0];

uint16_t sensor_measured[8] = [0];


// define pins
const int left_nslp_pin =31;     // nslp ==> awake & ready for PWM
const int right_nslp_pin=11;    // nslp ==> awake & ready for PWM
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
    // subtract 8 minimums
    // ratio 8 values to 1000
    // use 8 weights to compute error
    int error = compute_error();
    // compute steering change command
    // add change to one wheel, subtract from other
}

// HELPER FUNCTIONS

// computes the error term by fusing each value according to values in sensor_weight
int compute_error (int sens[8]){
    // find min
    return       (sens[0]*sensor_weight[0] 
                + sens[1]*sensor_weight[1]
                + sens[2]*sensor_weight[2]
                + sens[3]*sensor_weight[3]
                + sens[4]*sensor_weight[4]
                + sens[5]*sensor_weight[5]
                + sens[6]*sensor_weight[6]
                + sens[7]*sensor_weight[7])/8;
}

// computes steering change according to error determined by weights on track
void adjust_steeri