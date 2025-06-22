#ifndef CONFIG_H
#define CONFIG_H

// --- Generelle Indstillinger ---
// #define LOOP_TIME_MS 15 // Freq = 1000/15 = 66.7 Hz - FJERNES, loop kører så hurtigt som muligt
#define MAX_TILT_ANGLE_SAFETY 30.0 // Grader. Robotten stopper hvis den vipper mere end dette
#define RECOVERY_ANGLE_THRESHOLD 5.0 // Grader, tærskel for at gå fra FALLEN til BALANCING <-- TILFØJET
#define ANTI_WINDUP_ANGLE_THRESHOLD 15.0 // Grader, tærskel for I-term akkumulering <-- TILFØJET


// --- Motor Indstillinger ---
#define MAX_RPM 300.0 // Maksimalt forventet/ønsket RPM for motoren
// PWM indstillinger (ESP32 har 16 uafhængige kanaler)
#define PWM_FREQ 5000    // PWM frekvens i Hz
#define PWM_RESOLUTION 8 // PWM opløsning i bits (f.eks. 8 for 0-255)
#define PWM_MAX_DUTY ((1 << PWM_RESOLUTION) - 1) // Beregner 2^resolution - 1
#define DEADZONE_RPM_THRESHOLD 1
#define MIN_EFFECTIVE_RPM 10 
#define MIN_MOTOR_PWM 20
#define WHEEL_DIAMETER_M 0.13
#define SAVE_THRESHOLD 0.01 // Grænse for ændring i g_init_balance før gemning
#define STABILITY_ITERM_THRESHOLD 2
#define STABILITY_MIN_DURATION_MS 5000 // 5 dekunder
#define MIN_BALANCE_TIME_BEFORE_SAVE_CHECK_MS 10000 // 10 dekunder

// Minimum tid i ms for at måle motorhastighed (RPM).
// Hvis der går længere tid uden pulser, antages 0 RPM.
// Juster baseret på encoder/motor. F.eks. 20ms.
#define MOTOR_MIN_MEASUREMENT_TIME_MS 20 // <-- SIKRE DEN ER DEFINERET

#define LOWPASSFILTER(input, output, alpha) ((alpha * input) + ((1.0 - alpha) * output))
#define ALPHA 0.8

#define IMU_STARTUP_READ_ATTEMPTS 10 // Antal forsøg på at læse IMU ved opstart
#define IMU_STARTUP_READ_DELAY_MS 50 // Forsinkelse mellem forsøg i ms

// --- PID Konstanter (Manuelt Styret) ---
// VIGTIGT: Disse er GLOBALE og vil blive overskrevet af NVS ved opstart.
// De defineres i tuning_handler.cpp
extern double g_balance_kp;
extern double g_balance_ki;
extern double g_balance_kd;
extern double g_velocity_kp;
extern double g_init_balance; // Mål-vinkel for balance (typisk tæt på 0, men kan være let off-set)
extern double g_balance_output_to_rpm_scale; // Skaleringsfaktor fra PID output til RPM kommando
extern double g_power_gain; // Multiplier for at øge motorkraften ved større hældning

// Begrænsning for I-term anti-windup og/eller PID output generelt
// Dette er en grænse i 'raw_pid_output' skalaen, før scaling/gain
#define BALANCE_PID_OUTPUT_LIMIT 100.0 // Eksempelværdi, juster efter behov

// --- IMU Indstillinger (BNO085) ---
// Raport interval i mikrosekunder. SH2 anbefaler 5ms (5000us) for Rotation Vector.
// Vi kan køre hurtigere hvis nødvendigt, men start med 5-10ms.
// Da vores main loop kører i sin egen takt, læser vi bare reports whenever available.
// Intervallet her bestemmer hvor OFTE sensoren sender data.
#define IMU_REPORT_INTERVAL_US 5000 // 5ms <-- TILFØJET

// Tuning handler: Deklarer g_enable_csv_output som extern, hvis den defineres i .cpp
extern bool g_enable_csv_output; // <-- SIKRE DEN ER DEFINERET

enum RobotState {
  IDLE,
  CALIBRATING_IMU,
  BALANCING,
  FALLEN
};


#endif // CONFIG_H