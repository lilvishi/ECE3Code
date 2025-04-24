 //PWM Period Experiment
 
/* This code does the following:
  - create a PWM signal that runs the left motor
    - use bumper switch 0 (BMP0) to select from two PWM signal periods
    - the signal duty cycle is set to 50%
  - calculate the speed of the left motor using the encoder
    - this measurement is in arbitrary units (details below)
  - serial communication sends this information to your computer
    - serial baud rate is set to 19200 (increased data rate from 9600)
    - output format is such that the "Serial Plotter" can be used to view data
 */

#include <ECE3.h> // Used for encoder functionality

/* NOTE: We use the functions micros() and getEncoderCount_left()
       - micros()
           returns how many microseconds it has been since Launchpad startup
       - getEncoderCount_left()
           returns how many encoder increments the left wheel has turned
       - resetEncoderCount_left()
           sets encoder increments to 0   */

// Pin Assignments            // commented are right side pins
const int left_nslp_pin = 31; //11; 
const int left_dir_pin  = 29; //30; 
const int left_pwm_pin  = 40; //39;
const int bump_sw_0_pin = 24;
const int LED_RF        = 41; // LED Red Front
const int user_sw_2_pin=74;


// PWM Signal Variables
bool bump_sw_0_reading;
bool pwm_state;
unsigned long set_period;                  // Current setting for period (set by BMP0)
unsigned long t_last_period;               // The time the last period ended
unsigned long d_period;                    // The elapsed time since the current period began
const double  duty_cycle         = 0.50;
const unsigned long long_period  = 2000e3; // 1 s
const unsigned long short_period = 200e3;  // 100 ms

// Encoder Variables
unsigned long t_enc_cnt_prev;           // The time of at the end of the last bin
unsigned long enc_bin_cnt;              // The change in encoder count in the last bin (aka motor speed)
const unsigned long enc_bin_len = 50e3; // 50 ms bins
    // Encoder Speed Calculation Explanation:
    // We wait for a set amount of time (enc_bin), and find how many
    // times the encoder has incremented in that period. We call 
    // this period a bin when refering to the encoder. The number 
    // encoder counts per bin is a proportional to speed.

// Misc Variables
unsigned long curr_micros; // Stores micros() reading for each execution of loop()
int user_sw_2_reading;

///////////////////////
void setup() {
// This function runs once

  // Pin Settings
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(bump_sw_0_pin,INPUT_PULLUP);
  pinMode(LED_RF, OUTPUT);
  pinMode(user_sw_2_pin,INPUT_PULLUP);


  // Setting Initial Values
  //digitalWrite(left_nslp_pin, HIGH); // we don't set this since we want the motor to coast
      // NOTE: For the purposes of this experiment, we drive
      //       left_nslp_pin with a PWM signal as well. This
      //       allows the motor to coast, rather than brake,
      //       when the PWM signal is low. This only matters
      //       for large periods.
  digitalWrite(left_dir_pin,LOW);

  t_last_period = micros();

  ECE3_Init(); // Used for encoder functionality
  resetEncoderCount_left();
  enc_bin_cnt = 0;
  t_enc_cnt_prev = micros();

  Serial.begin(9600); 
  
  delay(2000); // Wait 2 seconds before starting 
}

void loop() {
// This function loops forever
  user_sw_2_reading = digitalRead(user_sw_2_pin); 
  // Determine set_period from BMP0
  bump_sw_0_reading = digitalRead(bump_sw_0_pin);
  if(bump_sw_0_reading)
//  if(user_sw_2_reading)
    set_period = long_period;
  else
    set_period = short_period;

  // Determine where in a period we are
  curr_micros = micros();
  d_period = curr_micros - t_last_period;

  // Set the PWM signal state
  pwm_state = d_period < set_period * duty_cycle;

  // Write the PWM signal state to the left motor
  digitalWrite(left_nslp_pin, pwm_state); // as noted before, this allows the left motor to coast
  digitalWrite(left_pwm_pin, pwm_state);
  digitalWrite(LED_RF, pwm_state);

  // Check if the period has ended
  if (d_period > set_period)
    t_last_period = curr_micros;



  // Encoder Speed Calculation (SEE EXPLANATION AT VARIABLE DECLARATIONS)
  if(curr_micros - t_enc_cnt_prev > enc_bin_len) { // have we finished a bin?
    enc_bin_cnt = getEncoderCount_left();          // find encoder ticks in the recent bin (calculate speed)
    resetEncoderCount_left();
    t_enc_cnt_prev = curr_micros;
  }



  // Print to the Serial Plotter
  Serial.print("X1:");  
  Serial.print(255 * pwm_state); // scaling up pwm_state by 255 so it is visible on the same plot as enc_cnt_diff
  Serial.print(",");
  Serial.print("X2:"); 
  Serial.println(enc_bin_cnt);
  delay(50);
 
}

