#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include "config.h" // Til tuning konstanter, limits etc.

// Antag RobotState enum er defineret i config.h
// enum RobotState { IDLE, CALIBRATING_IMU, BALANCING, FALLEN }; // <-- DEFINERET I config.h


class BalanceController {
private:
    // Manuel integral akkumulator
    double _pitch_error_integral;

public: // Gjort public for nem adgang til logging fra main.cpp
    double pTerm_log = 0.0;
    double iTerm_log = 0.0;
    double dTerm_log = 0.0;
    double balanceCmd_log = 0.0; // Det samlede RÅ kontrol output (efter constrain)

    // Dette er værdien FØR styring, som repræsenterer den vertikale kontrol.
    double scaled_output_log = 0.0;
    double getIntegral() const { return _pitch_error_integral; }

private:
    // Private kopier af tuning variabler (hvis du ikke vil bruge de globale direkte)
    // Men da de er globale fra NVS, er det simplere at bruge 'extern' variablerne
    // Direkte her, ligesom i main.cpp.


public:
    BalanceController(); // Konstruktør

    // Initialisering - nulstil integral etc.
    void begin();

    // Hovedkontrolfunktion
    // Beregner PID output baseret på inputs og returnerer target RPM for hver motor.
    // currentPitch: Aktuel pitch vinkel fra IMU (grader)
    // currentPitchRate: Aktuel pitch vinkelhastighed fra IMU (grader/sek) - bruger rå
    // currentVelocity: Aktuel fremad/tilbage hastighed (f.eks. gennemsnit RPM fra motorer)
    // dt: Tid siden sidste opdatering (sekunder)
    // steeringCmd: Styringskommando (f.eks. fra joystick)
    // state: Den aktuelle robot state (BALANCING, FALLEN, etc.)
    // targetRpmLeft, targetRpmRight: Output parametre til de beregnede RPM værdier
    void update(double currentPitch, double currentPitchRate, double currentVelocity,
                double dt, double steeringCmd, RobotState currentState,
                double& targetRpmLeft, double& targetRpmRight);

    // Nulstil integral termen (f.eks. ved state skift)
    void resetIntegral();

    // Getters til logning (alternativ til public medlemmer)
    // double getPTerm() const { return pTerm_log; }
    // double getITerm() const { return iTerm_log; }
    // double getDTerm() const { return dTerm_log; }
    // double getBalanceCmd() const { return balanceCmd_log; }
    // double getScaledOutput() const { return scaled_output_log; } // <--- Getter til scaled_output_log
};

#endif // BALANCE_CONTROLLER_H