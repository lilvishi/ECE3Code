// project_car.
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

// define variables + arrays
const int sensor_weight[8] = [-15,-14,-12,-8,8,12,14,15];
int sensor_measured[8] = [0];

// define pins

// SETUP (program run once for initialization)
void setup(){
    // set pin modes 
    // initialize pins
    // initialize serial communication
    // set base speed
}

// LOOP (program run continiously as car is on)
void loop(){
    // read sensor Values
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
void adjust_steering(int error){

}