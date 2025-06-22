// --------------- FIL: include/Motor.h ---------------
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h" // For MOTOR_MIN_MEASUREMENT_TIME_MS etc.

#define RPM_TIMEOUT_MS 500 // Timeout for RPM måling (hvis ingen pulser i denne periode, antag RPM=0)
#define COUNTS_PER_REV 16 // Antal pulser pr. omdrejning (MOTORAKSEL) - Aktiveret igen
#define GEAR_RATIO 43.7f  // Gearforhold motor til hjul (motor drejer GEAR_RATIO gange hurtigere end hjul) - Aktiveret igen

class Motor {
private:
    int _pinIN1;
    int _pinIN2;
    int _pinENA;
    int _hallPinA;
    int _pwmChannel;
    int _pwmMax; // Vil blive sat til PWM_MAX_DUTY fra config.h
    int _minMeasurementTimeMs;

    volatile unsigned long _pulseCount = 0;
    int _actualRpm = 0;
    unsigned long _lastRpmUpdateTime = 0;
    unsigned long _startMeasurementTime = 0; // Bruges til at måle tidsinterval for pulser
    bool _currentDirectionForward = true;  // Default retning

public:
    // Konstruktør - initialiserer motor med pins, PWM kanal og minimum måletid for RPM
    Motor(int pinIN1, int pinIN2, int pinENA, int hallPinA, int pwmChannel, int minMeasurementTimeMs = MOTOR_MIN_MEASUREMENT_TIME_MS);

    // Initialiserer motor pins og PWM
    void begin();

    // Sætter motorens rotationsretning
    // forward = true for fremad, false for tilbage
    void setDirection(bool forward);

    // Stopper motoren (sætter PWM til 0)
    void stop();

    // Anvender en rå PWM værdi direkte til motoren (0 til _pwmMax)
    // Retning skal være sat korrekt FØR kald af denne funktion.
    void applyRawPwm(int pwm);

    // Returnerer den senest beregnede RPM værdi for motoren (med fortegn for retning)
    // Opdaterer internt RPM før returnering. Sætter RPM til 0 ved timeout.
    int getActualRpm();

    // ISR funktion: Inkrementerer pulstælleren. Skal kaldes fra en ekstern interrupt.
    void IRAM_ATTR incrementPulseCount();

    // Nulstiller pulstælleren og starttiden for en ny RPM måling.
    void resetPulseCount();
};

#endif // MOTOR_H