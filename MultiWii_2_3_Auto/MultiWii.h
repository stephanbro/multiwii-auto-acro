#ifndef MULTIWII_H_
#define MULTIWII_H_

#define  VERSION  230

#include "types.h"

#define SJB_AUTON
//#define SJB_AUTON_TEST

#ifdef SJB_AUTON
  #define PROC_SPEED 1000000
  #define ALT_ERROR_DEADBAND 30
  
  //Throttle Defines
  #define THROTTLE_FULL 2000        //power when doing the flip climb
  #define THROTTLE_OFF 1000
  #define THROTTLE_HOVER 1500
  #define THROTTLE_LOW 1200
  #define THROTTLE_MID 1500

  //Climb Defines
  #define CLIMB_VARIO 100        //Climb speed in cm/s, max is 250cm/s
  #define FLIP_ALT_BUFFER 200    //Altitude above hold to climb before starting flip
  #define ALT_REACHED_CYCLES 50  //Number of times to climb to goal alt, used to reduce noise, 50 = 1 second

  //Hold Defines
  #define HOLD_STATE_ALT 700     //500 had a good alt on low power 500=500cm?
  #define HOLD_TIME_CYCLES 1000  //Time to stay in intermediate state in millisec
  #define ALT_DEADBAND 100       //within +/- deadband in cm is on target, to account for noise

  //Flip Defines
  #define MAX_FLIP_COUNT 2        //How many flips to be done
  #define FLIP_ANGLE_CUTOUT 90   //Angle to cut the power at in degrees
  #define FLIP_ANGLE_HOLD 90     //Angle to go to Hold State - 10.0deg=100
  #define FLIPPED 1

  //Land Defines
  #define AUTOLAND_SAFETY_ALT 300 //Altitude to change between fast and slow vario
  #define AUTOLAND_FAST_VARIO 100 //Fast desc speed in cm/s
  #define AUTOLAND_SLOW_VARIO 50  //Slow desc speed in cm/s
  #define ALT_ERROR_CUTOFF 10000
  #define BARO_PID_CUTOFF -250

  //State Defines
  #define INIT 1
  #define CLIMB 2
  #define HOLD 3       //Intermediate State
  #define FLIP_CLIMB 4 //Climb before Flip State
  #define FLIP 5       //Flip State
  #define FLIP_RECOVER 6
  #define LAND 7       //Land State

  //Other Defines
  #define RECOVER_CYCLES 2000 //time to run full power in millisecs
  #define FALL_CYCLES 25
  #define STATE_SWITCH 1970
  #define TRAVEL_HOME_LATCH 6 //in meters?
  #define TRAVEL_HOME_UNLATCH 1 //in meters?
  
  static int16_t targetVario = 0;

#endif
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;

// default POSHOLD control gains
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //
#define NAV_IMAX               20        // degrees

#define MINCHECK 1115
#define MAXCHECK 1900

extern volatile unsigned long timer0_overflow_count;

extern const char pidnames[];
extern const char boxnames[];
extern const uint8_t boxids[];

extern uint32_t currentTime;
extern uint16_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t  magHold,headFreeModeHold;
extern uint8_t  vbatMin;
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern int32_t  AltHold;
extern int16_t  sonarAlt;
extern int16_t  BaroPID;
extern int16_t  errorAltitudeI;

extern int16_t  i2c_errors_count;
extern uint8_t alarmArray[16];
extern global_conf_t global_conf;

extern imu_t imu;
extern analog_t analog;
extern alt_t alt;
extern att_t att;
#ifdef LOG_PERMANENT
extern plog_t plog;
#endif

extern int16_t debug[4];

extern conf_t conf;

extern int16_t  annex650_overrun_count;
extern flags_struct_t f;
extern uint16_t intPowerTrigger1;

extern int16_t gyroZero[3];
extern int16_t angle[2];


#if BARO
  extern int32_t baroPressure;
  extern int32_t baroTemperature;
  extern int32_t baroPressureSum;
#endif

extern int16_t axisPID[3];
extern int16_t motor[8];
extern int16_t servo[8];

extern int16_t failsafeEvents;
extern volatile int16_t failsafeCnt;

extern int16_t rcData[RC_CHANS];
extern int16_t rcSerial[8];
extern int16_t rcCommand[4];
extern uint8_t rcSerialCount;
extern int16_t lookupPitchRollRC[5];
extern int16_t lookupThrottleRC[11];

#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
  #define PMOTOR_SUM 8                     // index into pMeter[] for sum
  extern uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  extern uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  extern uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  extern uint16_t powerValue;              // last known current
#endif

#if defined(LCD_TELEMETRY)
  extern uint8_t telemetry;
  extern uint8_t telemetry_auto;
#endif
#ifdef LCD_TELEMETRY_STEP
  extern char telemetryStepSequence[];
  extern uint8_t telemetryStepIndex;
#endif

#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  extern uint16_t cycleTimeMax;       // highest ever cycle timen
  extern uint16_t cycleTimeMin;       // lowest ever cycle timen
  extern int32_t  BAROaltMax;         // maximum value
  extern uint16_t GPS_speedMax;       // maximum speed from gps
  extern uint16_t powerValueMaxMAH;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  extern uint32_t armedTime;
#endif


  // **********************
  // GPS common variables
  // **********************
  extern int16_t  GPS_angle[2];                      // the angles that must be applied for GPS correction
  extern int32_t  GPS_coord[2];
  extern int32_t  GPS_home[2];
  extern int32_t  GPS_hold[2];
  extern uint8_t  GPS_numSat;
  extern uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  extern int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  extern uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  extern uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  extern uint8_t  GPS_update;                              // a binary toogle to distinct a GPS position update
  extern uint16_t GPS_ground_course;                       //                   - unit: degree*10
  extern uint8_t  GPS_Present;                             // Checksum from Gps serial
  extern uint8_t  GPS_Enable;

  #define LAT  0
  #define LON  1

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2
  extern uint8_t nav_mode; // Navigation mode
  extern int16_t  nav[2];
  extern int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  // default POSHOLD control gains
  #define POSHOLD_P              .11
  #define POSHOLD_I              0.0
  #define POSHOLD_IMAX           20        // degrees

  #define POSHOLD_RATE_P         2.0
  #define POSHOLD_RATE_I         0.08      // Wind control
  #define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
  #define POSHOLD_RATE_IMAX      20        // degrees

  // default Navigation PID gains
  #define NAV_P                  1.4
  #define NAV_I                  0.20      // Wind control
  #define NAV_D                  0.08      //
  #define NAV_IMAX               20        // degrees

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2

  extern volatile uint8_t  spekFrameFlags;
  extern volatile uint32_t spekTimeLast;

  #if defined(OPENLRSv2MULTI)
    extern uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
  #endif

  // **********************
  //Automatic ACC Offset Calibration
  // **********************
  #if defined(INFLIGHT_ACC_CALIBRATION)
    extern uint16_t InflightcalibratingA;
    extern int16_t AccInflightCalibrationArmed;
    extern uint16_t AccInflightCalibrationMeasurementDone;
    extern uint16_t AccInflightCalibrationSavetoEEProm;
    extern uint16_t AccInflightCalibrationActive;
  #endif

#if defined(ARMEDTIMEWARNING)
  extern uint32_t  ArmedTimeWarningMicroSeconds;
#endif

#if defined(THROTTLE_ANGLE_CORRECTION)
  extern int16_t throttleAngleCorrection;
  extern int8_t  cosZ;
#endif

void annexCode();

#endif /* MULTIWII_H_ */
