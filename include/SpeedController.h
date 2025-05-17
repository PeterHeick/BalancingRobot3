#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include "Motor.h"     // Grundlæggende motor klasse
#include <PID_v1.h>    // Arduino PID bibliotek
#include "config.h"    // For konstanter

// Sørg for at RPM_LOOKUP_TABLE_SIZE er defineret i config.h
#ifndef RPM_LOOKUP_TABLE_SIZE
#error "RPM_LOOKUP_TABLE_SIZE skal være defineret i config.h (MAX_RPM + 1)"
#endif

class SpeedController {
public:
    /**
     * @brief Konstruktør.
     * @param motor Reference til det Motor objekt der skal styres.
     * @param lookupTable Pointer til den hardkodede RPM->PWM tabel for denne motor.
     * @param Kp Proportional gain for hastigheds PID.
     * @param Ki Integral gain for hastigheds PID.
     * @param Kd Derivative gain for hastigheds PID.
     */
    SpeedController(Motor &motor, const int* lookupTable, double Kp, double Ki, double Kd);

    /**
     * @brief Initialiserer PID controlleren (skal kaldes fra setup()).
     */
    void begin();

    /**
     * @brief Sætter den ønskede mål-RPM for motoren.
     * @param targetRpm Den ønskede hastighed i RPM (kan være positiv eller negativ for retning).
     */
    void setTargetRpm(double targetRpm);

    /**
     * @brief Opdaterer hastighedskontrollen. Skal kaldes i hvert loop-interval.
     * Læser aktuel RPM, beregner PID korrektion, slår op i tabel,
     * kombinerer og sender endelig PWM til motoren.
     */
    void update(int fortegn);

    /**
     * @brief Stopper motoren øjeblikkeligt (sætter PWM til 0).
     */
    void stop();

    /**
     * @brief Returnerer den senest målte (evt. filtrerede) RPM.
     */
    double getActualRpm() const;

    /**
     * @brief Returnerer den senest beregnede endelige PWM værdi.
     */
    int getLastPwm() const;

    /**
     * @brief Giver mulighed for at ændre PID tunings under kørsel.
     */
    void setTunings(double Kp, double Ki, double Kd);


private:
    Motor& _motor; // Reference til den fysiske motor
    PID _speedPID; // PID controller objekt

    // PID variabler
    double _pidInput;      // Input til PID (målt/filtreret RPM)
    double _pidOutput;     // Output fra PID (korrektions-PWM +/-)
    double _pidSetpoint;   // Ønsket mål-RPM (altid positiv til PID)

    // Pointer til den hardkodede opslagstabel (RPM -> PWM)
    const int* _rpmToPwmTable;

    // PID parametre (gemmes lokalt)
    double _kp;
    double _ki;
    double _kd;

    // Tilstand for filtrering
    double _previousFilteredRpm;
    bool _filterInitialized;

    // Sidst anvendte PWM
    int _lastAppliedPwm;
};

#endif // SPEEDCONTROLLER_H