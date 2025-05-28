#include "SpeedController.h"
#include <Arduino.h> // For constrain, abs, osv. (stadig nødvendig her)
#include "config.h"  // For diverse konstanter

// FJERN DISSE LINJER - De hører til BNO085 initialisering, ikke SpeedController
/*
#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif

bool bno085Initialized = false; // <-- FJERN
*/

// --- Konstruktør ---
SpeedController::SpeedController(Motor &motor) :
    _motor(motor), // Gem reference til motor
    _lastAppliedPwm(0)
{
    // Ingen PID eller feedforward tables her
}

// --- Initialisering ---
void SpeedController::begin() {
    _lastAppliedPwm = 0;
    _motor.applyRawPwm(0); // Start med stop
}

// --- Sæt Direkte PWM Baseret på Target RPM ---
// targetRpm kommer fra hoved-balance PID'en i main.cpp
void SpeedController::setTargetRpm(double targetRpm) {

    // Håndter retning
    bool forward = targetRpm >= 0;
    _motor.setDirection(forward);

    // Konverter RPM til PWM med simpel skalering
    double absRpm = abs(targetRpm);

    // Deadzone - under denne RPM sender vi 0 PWM
    // Dette forhindrer lav-hastighed spjæt for tæt på balancepunktet.
    if (absRpm < DEADZONE_RPM_THRESHOLD) { // Bruger konstant fra config.h
        _lastAppliedPwm = 0;
        _motor.applyRawPwm(0);
        // Reset Integralen i main.cpp hvis motorerne er stoppet? Overvej.
        // Typisk nulstiller man integralen, når output constraines til 0.
        // Dette skal håndteres i main.cpp's PID loop hvis nødvendigt.
        return; // Stop her, send 0 PWM
    }

    int pwm;
    // Pålidelig RPM start ved MIN_EFFECTIVE_RPM, minimum PWM er MIN_MOTOR_PWM
    // Antag en lineær relation fra MIN_EFFECTIVE_RPM/MIN_MOTOR_PWM til MAX_RPM/255 (eller PWM_MAX_DUTY)
    // Du bruger 255 her, så vi beholder det for nu, men PWM_MAX_DUTY er mere generel.
    // For 8-bit PWM er 255 = PWM_MAX_DUTY.

    if (absRpm <= MIN_EFFECTIVE_RPM) {
        // RPM er i intervallet DEADZONE_RPM_THRESHOLD til MIN_EFFECTIVE_RPM
        // Sæt til minimum PWM for at sikre motoren overhovedet kan bevæge sig
        pwm = MIN_MOTOR_PWM; // Bruger konstant fra config.h
    } else {
        // RPM er over MIN_EFFECTIVE_RPM. Skaler lineært op til MAX_RPM
        // Map absRpm fra [MIN_EFFECTIVE_RPM, MAX_RPM] til [MIN_MOTOR_PWM, 255]
        // Værdi = StartOutput + (Input - StartInput) * (EndOutput - StartOutput) / (EndInput - StartInput)
        pwm = MIN_MOTOR_PWM +
              (absRpm - MIN_EFFECTIVE_RPM) * (255.0 - MIN_MOTOR_PWM) /
              (MAX_RPM - MIN_EFFECTIVE_RPM); // Bruger konstanter fra config.h
    }

    // Begræns PWM til gyldigt område (0 til 255 for 8-bit)
    // constrain(val, low, high)
    pwm = constrain(pwm, 0, 255); // Eller constrain(pwm, 0, PWM_MAX_DUTY); hvis defineret i config.h

    // Send til motor
    _lastAppliedPwm = pwm;
    _motor.applyRawPwm(pwm);
}

// --- Stop ---
void SpeedController::stop() {
    _motor.applyRawPwm(0);
    _lastAppliedPwm = 0;
}

// --- Getters ---
double SpeedController::getActualRpm() const {
    return _motor.getActualRpm(); // Direkte fra motor
}

int SpeedController::getLastPwm() const {
    return _lastAppliedPwm;
}

// --- Set Tunings (ikke længere nødvendig) ---
// Denne metode gør intet og kan fjernes helt.
/*
void SpeedController::setTunings(double Kp, double Ki, double Kd) {
    // Ingen PID længere - denne metode gør intet
}
*/