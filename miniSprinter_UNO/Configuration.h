#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Uncomment ONE of the next three lines - the one for your RepRap machine
//#define REPRAPPRO_MENDEL
//#define REPRAPPRO_WALLACE
#define CXP3DP

// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// MEGA/RAMPS up to 1.2  = 3,
// RAMPS 1.3 = 33
// Gen6 = 5,
// Sanguinololu up to 1.1 = 6
// Sanguinololu 1.2 and above, and Melzi = 62
// #define MOTHERBOARD 62
//
// Motherboard DEFINE not been used 
// PIN defination are listed at the end of this file
// CXP changed @ 25JAN2015



//// Thermistor settings:
// 1 is 100k thermistor
// 2 is 200k thermistor
// 3 is mendel-parts thermistor
// 4 is 10k thermistor
#define THERMISTORHEATER 1
#define THERMISTORBED 1

//// Calibration variables
// X, Y, Z, E steps per unit - Metric Prusa Mendel with Wade extruder:
//float axis_steps_per_unit[] = {91.4286, 91.4286,4000,910};

//// Calibration variables
// X, Y, Z, E steps per unit - RP3D.COM PANGGU with K extruder
//float axis_steps_per_unit[] = {99.8464, 99.8464,2560,164.3723}; //harryleo first 5 kits
//float axis_steps_per_unit[] = {85.3333, 85.3333,2560,130.8308};
float axis_steps_per_unit[] = {85.3333, 85.3333, 2560, 158.8308};

//// Endstop Settings
//#define ENDSTOPPULLUPS 1 // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool ENDSTOPS_INVERTING = true; //set to true to invert the logic of the endstops
//If your axes are only moving in one direction, make sure the endstops are connected properly.
//If your axes move in one direction ONLY when the endstops are triggered, set ENDSTOPS_INVERTING to true here

// This determines the communication speed of the printer
#define BAUDRATE 115200

// Comment out (using // at the start of the line) to disable SD support:
//#define SDSUPPORT 1

// Comment out (using // at the start of the line) to disable Bluetooth support:
//#define BLUETOOTH
#ifdef BLUETOOTH
#define BLUETOOTH_SERIAL Serial1
#endif


//// ADVANCED SETTINGS - to tweak parameters

//#include "thermistortables.h"

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

// Disables axis when it's not being used.
const bool DISABLE_X = false;
const bool DISABLE_Y = false;
const bool DISABLE_Z = true;
const bool DISABLE_E = false;

// Inverting axis direction
const bool INVERT_X_DIR = false;
const bool INVERT_Y_DIR = false;
const bool INVERT_Z_DIR = false;
const bool INVERT_E_DIR = true;

//// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

const bool min_software_endstops = false; //If true, axis won't move to coordinates less than zero.
const bool max_software_endstops = true;  //If true, axis won't move to coordinates greater than the defined lengths below.


const int X_MAX_LENGTH = 150;
const int Y_MAX_LENGTH = 150;
const int Z_MAX_LENGTH = 100;


/*#define Z_INCREMENT 0.05*/

//// MOVEMENT SETTINGS
const int NUM_AXIS = 4; // The axis order in all axis related arrays is X, Y, Z, E
float max_feedrate[] = {3000, 3000, 200, 10000};
float homing_feedrate[] = {1500, 1500, 200};
bool axis_relative_modes[] = {false, false, false, false};

// Min step delay in microseconds. If you are experiencing missing steps, try to raise the delay microseconds, but be aware this
// If you enable this, make sure STEP_DELAY_RATIO is disabled.
//#define STEP_DELAY_MICROS 1

// Step delay over interval ratio. If you are still experiencing missing steps, try to uncomment the following line, but be aware this
// If you enable this, make sure STEP_DELAY_MICROS is disabled. (except for Gen6: both need to be enabled.)
//#define STEP_DELAY_RATIO 0.25

// Comment this to disable ramp acceleration
//#define RAMP_ACCELERATION 1

//// Acceleration settings
#ifdef RAMP_ACCELERATION
// X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
float max_start_speed_units_per_second[] = {25.0, 25.0, 0.2, 10.0};
long max_acceleration_units_per_sq_second[] = {1000, 1000, 50, 250}; // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
long max_travel_acceleration_units_per_sq_second[] = {500, 500, 50, 500}; // X, Y, Z max acceleration in mm/s^2 for travel moves
#endif

// Machine UUID
// This may be useful if you have multiple machines and wish to identify them by using the M115 command.
// By default we set it to zeros.
const char uuid[] = "20BA-1127";


//// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

//// PID settings:
// Uncomment the following line to enable PID support. This is untested and could be disastrous. Be careful.
//#define PIDTEMP 1
#ifdef PIDTEMP
int pid_max = 255; // limits current to nozzle
int pid_i_max = 80;//130;//125;
double Kp = 2;//1.10;
double Ki = 0.01;
double Kd = 20.0;//0.01;
#endif

//M109 target window - machine will deem to have reached target temperature when nozzle reaches Temp = target - NZONE.
int nzone = 5;//2;

//#define DEBUG_PID

#define FAN_INIT 200

// How often should the heater check for new temp readings, in milliseconds
#define HEATER_CHECK_INTERVAL 112
#define BED_CHECK_INTERVAL 5000
// Comment the following line to enable heat management during acceleration
//#define DISABLE_CHECK_DURING_ACC
#ifndef DISABLE_CHECK_DURING_ACC
// Uncomment the following line to disable heat management during moves
//#define DISABLE_CHECK_DURING_MOVE
#endif
// Uncomment the following line to disable heat management during travel moves (and extruder-only moves, eg: retracts), strongly recommended if you are missing steps mid print.
// Probably this should remain commented if are using PID.
// It also defines the max milliseconds interval after which a travel move is not considered so for the sake of this feature.
//#define DISABLE_CHECK_DURING_TRAVEL 1000

//// Temperature smoothing - only uncomment this if your temp readings are noisy (Gen6 without EvdZ's 5V hack)
//#define SMOOTHING 1
//#define SMOOTHFACTOR 16 //best to use a power of two here - determines how many values are averaged together by the smoothing algorithm

//// Experimental watchdog and minimal temp
// The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds

//// The minimal temperature defines the temperature below which the heater will not be enabled
#define MINTEMP 5

//// Experimental max temp
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define MAXTEMP 300

// Select one of these only to define how the nozzle temp is read.
#define HEATER_USES_THERMISTOR
//#define HEATER_USES_AD595
//#define HEATER_USES_MAX6675

// Select one of these only to define how the bed temp is read.
#define BED_USES_THERMISTOR
//#define BED_USES_AD595

// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG
#ifdef DEBUG
//#define DEBUG_PREPARE_MOVE //Enable this to debug prepare_move() function
//#define DEBUG_BRESENHAM //Enable this to debug the Bresenham algorithm
//#define DEBUG_RAMP_ACCELERATION //Enable this to debug all constant acceleration info
//#define DEBUG_MOVE_TIME //Enable this to time each move and print the result
//#define DEBUG_HEAT_MGMT //Enable this to debug heat management. WARNING, this will cause axes to jitter!
//#define DEBUG_DISABLE_CHECK_DURING_TRAVEL //Debug the namesake feature, see above in this file
#endif

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://reprap.org/wiki/Thermistor)
// Made with the online thermistor table generator by nathan7 at http://calculator.josefprusa.cz/
// r0: 100000
// t0: 25
// r1: 0
// r2: 10000
// beta: 3950
// max adc: 1023
#define NUMTEMPS        260-211+1
#define OVERSAMPLENR    1
short temptable[NUMTEMPS][2] =
{
         {1*OVERSAMPLENR,        938},
         {31*OVERSAMPLENR,       314},
         {41*OVERSAMPLENR,       290},
         {51*OVERSAMPLENR,       272},
         {61*OVERSAMPLENR,       258},
         {71*OVERSAMPLENR,       247},
         {81*OVERSAMPLENR,       237},
         {91*OVERSAMPLENR,       229},
         {101*OVERSAMPLENR,      221},
         {111*OVERSAMPLENR,      215},
         {121*OVERSAMPLENR,      209},
         {131*OVERSAMPLENR,      204},
         {141*OVERSAMPLENR,      199},
         {151*OVERSAMPLENR,      195},
         {161*OVERSAMPLENR,      190},
         {171*OVERSAMPLENR,      187},
         {181*OVERSAMPLENR,      183},
         {191*OVERSAMPLENR,      179},
         {201*OVERSAMPLENR,      176},
         {221*OVERSAMPLENR,      170},
         {241*OVERSAMPLENR,      165},
         {261*OVERSAMPLENR,      160},
         {281*OVERSAMPLENR,      155},
         {301*OVERSAMPLENR,      150},
         {331*OVERSAMPLENR,      144},
         {361*OVERSAMPLENR,      139},
         {391*OVERSAMPLENR,      133},
         {421*OVERSAMPLENR,      128},
         {451*OVERSAMPLENR,      123},
         {491*OVERSAMPLENR,      117},
         {531*OVERSAMPLENR,      111},
         {571*OVERSAMPLENR,      105},
         {611*OVERSAMPLENR,      100},
         {641*OVERSAMPLENR,      95},
         {681*OVERSAMPLENR,      90},
         {711*OVERSAMPLENR,      85},
         {751*OVERSAMPLENR,      79},
         {791*OVERSAMPLENR,      72},
         {811*OVERSAMPLENR,      69},
         {831*OVERSAMPLENR,      65},
         {871*OVERSAMPLENR,      57},
         {881*OVERSAMPLENR,      55},
         {901*OVERSAMPLENR,      51},
         {921*OVERSAMPLENR,      45},
         {941*OVERSAMPLENR,      39},
         {971*OVERSAMPLENR,      28},
         {981*OVERSAMPLENR,      23},
         {991*OVERSAMPLENR,      17},
         {1001*OVERSAMPLENR,     9},
         {1021*OVERSAMPLENR,     -27}
};

#define BNUMTEMPS       NUMTEMPS
#define bedtemptable    temptable



/****************************************************************************************
* pin assignment
*
****************************************************************************************/

#define X_STEP_PIN         2
#define X_DIR_PIN          3

#define Y_STEP_PIN         4
#define Y_DIR_PIN          5

#define Z_STEP_PIN         6
#define Z_DIR_PIN          7

#define E_STEP_PIN         8
#define E_DIR_PIN          9


#define X_ENABLE_PIN       10
#define Y_ENABLE_PIN       10
#define Z_ENABLE_PIN       11
#define E_ENABLE_PIN       11

#define Z_MIN_PIN     12
#define X_MIN_PIN     12
#define Y_MIN_PIN     12

#define Z_MAX_PIN    -1
#define Y_MAX_PIN    -1
#define X_MAX_PIN    -1


/*#define PROBE_PIN          11*/

#define LED_PIN            -1
#define FAN_PIN            13

#define PS_ON_PIN    -1
#define KILL_PIN     -1

#define HEATER_0_PIN       14 // (extruder)
#define HEATER_1_PIN       15 // (bed)

#define TEMP_0_PIN          16   // 14=A0  MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define TEMP_1_PIN          17   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)

// UNO
// A0 14
// A1 15
// A2 16        EXTURDER
// A3 17        BED
// A4 18
// A5 19




#endif
