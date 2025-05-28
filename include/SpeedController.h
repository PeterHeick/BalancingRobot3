#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include "Motor.h" // Antag at du har en Motor klasse
#include "config.h" // For konstanter som MAX_RPM, MIN_MOTOR_PWM etc. <-- Tilføjet for klarhed

class SpeedController {
public:
    // SIMPLIFICERET konstruktør - ingen PID eller lookup tables
    SpeedController(Motor &motor);

    // Initialisering
    void begin();

    // Sæt target RPM (konverteres direkte til PWM)
    void setTargetRpm(double targetRpm);

    // Update (nu meget simpel eller tom) - FJERNES
    // void update(int fortegn = 1);

    // Stop motor
    void stop();

    // Getters
    double getActualRpm() const;
    int getLastPwm() const;

    // Set tunings (ikke længere relevant) - FJERNES
    // void setTunings(double Kp, double Ki, double Kd);

private:
    Motor &_motor;              // Reference til motor objekt
    int _lastAppliedPwm;        // Sidste PWM værdi sendt til motor

    // FJERNET: Alle PID relaterede medlemmer
    // FJERNET: Feedforward lookup table
    // FJERNET: Filter variabler
};

#endif // SPEEDCONTROLLER_H