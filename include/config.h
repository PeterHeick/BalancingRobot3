#ifndef config_h
#define config_h

#include <Arduino.h>

// ---- Globale Tuning Variable (deklarationer) ----
// Disse vil blive defineret i f.eks. tuning_handler.cpp
extern double g_balance_kp;
extern double g_balance_ki;
extern double g_balance_kd;
extern double g_init_balance;
extern double g_balance_output_to_rpm_scale;
extern double g_power_gain;

// ---- Andre Konstanter (kan forblive #define) ----
#define MAXPITCH 10
// #define MAX_TILT_ANGLE MAXPITCH

// Speed PID konstanter (hvis du ikke vil tune dem via seriel lige nu)
#define SPEED_KP 0.9
#define SPEED_KI 4.0
#define SPEED_KD 0.0
#define SPEED_PID_CORRECTION_MIN -50
#define SPEED_PID_CORRECTION_MAX 50

#define MIN_MOTOR_PWM 20        
#define MIN_EFFECTIVE_RPM 10    

#define DIRECTION_MAX_LENGTH 10

// Lowpass filter definition
#define LOWPASSFILTER(input, output, alpha) ((alpha * input) + ((1.0 - alpha) * output))
#define ALPHA 0.1 // Overvej om denne stadig bruges, eller om specifikke alpha'er er bedre
#define RPM_FILTER_ALPHA 0.1
#define BALANCE_FILTER_ALPHA 0.3 // Denne bruges i balanceInput

#define LOOP_TIME_MS 15
#define BALANCE_PID_OUTPUT_LIMIT 100
#define MAX_TILT_ANGLE_SAFETY 20
#define MAX_RPM 256
#define DEADZONE_RPM_THRESHOLD 1 // Sat til 1 som diskuteret
#define RPM_LOOKUP_TABLE_SIZE (MAX_RPM + 1)

#endif