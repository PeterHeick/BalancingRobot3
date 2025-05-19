#include "SpeedController.h"
#include <Arduino.h> // For constrain, abs, osv.
#include "config.h"  // For diverse konstanter

// --- Konstruktør ---
SpeedController::SpeedController(Motor &motor, const int* lookupTable, double Kp, double Ki, double Kd) :
    _motor(motor), // Gem reference til motor
    // Opret PID objekt. Input/Output/Setpoint peger på klassens medlemmer.
    // Retning (DIRECT) og SampleTime sættes i begin().
    _speedPID(&_pidInput, &_pidOutput, &_pidSetpoint, Kp, Ki, Kd, DIRECT),
    _pidInput(0.0),
    _pidOutput(0.0),
    _pidSetpoint(0.0),
    _rpmToPwmTable(lookupTable), // Gem pointer til den hardkodede tabel
    _kp(Kp),
    _ki(Ki),
    _kd(Kd),
    _previousFilteredRpm(0.0),
    _filterInitialized(false),
    _lastAppliedPwm(0)
{
    // Tjek om lookupTable er gyldig?
    if (_rpmToPwmTable == nullptr) {
        Serial.println("FEJL: SpeedController modtog nullptr til lookupTable!");
        // Evt. håndter fejl her - stop programmet?
    }
}

// --- Initialisering ---
void SpeedController::begin() {
    // Sæt PID sample time til det samme som hoved-loopet
    _speedPID.SetSampleTime(LOOP_TIME_MS);

    // Sæt PID output limits til korrektions-området
    // Disse konstanter skal defineres i config.h!
    _speedPID.SetOutputLimits(SPEED_PID_CORRECTION_MIN, SPEED_PID_CORRECTION_MAX);

    // Start PID'en
    _speedPID.SetMode(AUTOMATIC);
    _pidInput = _motor.getActualRpm(); // Læs start RPM
    _previousFilteredRpm = _pidInput;  // Initialiser filter
    _filterInitialized = true;
}

// --- Sæt Mål RPM ---
void SpeedController::setTargetRpm(double targetRpm) {
    // Sæt motorens retning baseret på fortegnet
    _motor.setDirection(targetRpm >= 0);

    // Sæt PID setpoint til den absolutte værdi
    _pidSetpoint = abs(targetRpm);

    // Begræns setpoint til tabelstørrelsen (selvom opslag også begrænser)
    if (_pidSetpoint > MAX_RPM) {
        _pidSetpoint = MAX_RPM;
    }
}

// --- Opdater Kontrol Loop ---
void SpeedController::update(int fortegn) {
    // 1. Læs rå RPM fra motor
    double rawRpmInput = _motor.getActualRpm();

    // 2. Anvend lavpasfilter (valgfrit men anbefalet)
    if (!_filterInitialized) { // Initialiser filter ved første kørsel
         _previousFilteredRpm = rawRpmInput;
         _filterInitialized = true;
    }
    // RPM_FILTER_ALPHA skal defineres i config.h
    _pidInput = LOWPASSFILTER(rawRpmInput, _previousFilteredRpm, RPM_FILTER_ALPHA);
    _previousFilteredRpm = _pidInput; // Gem til næste iteration

    // 3. Håndter situationen hvis målet er under dødzone-grænsen
    //    DEADZONE_RPM_THRESHOLD skal defineres i config.h (f.eks. 5 eller 10)
    if (_pidSetpoint < DEADZONE_RPM_THRESHOLD) {
        _speedPID.SetMode(MANUAL); // Slå PID fra for at undgå integral windup
        _pidOutput = 0;          // Ingen korrektion
        _lastAppliedPwm = 0;     // Send 0 PWM
        _motor.applyRawPwm(0);
        return; // Afslut update her
    }

    // Hvis vi er over dødzone, sørg for PID er tændt
    if (_speedPID.GetMode() == MANUAL) {
        _speedPID.SetMode(AUTOMATIC);
    }

    // 4. Feed-Forward: Slå base PWM op i den hardkodede tabel
    //    Konverter setpoint (dobbelt) til et heltal index
    int rpmIndex = constrain((int)round(_pidSetpoint), 0, MAX_RPM);
    // Hent PWM fra tabellen (som blev givet i konstruktøren)
    int basePwm = (_rpmToPwmTable != nullptr) ? _rpmToPwmTable[rpmIndex] : 0;

    // 5. Kør PID for at få korrektion (+/-)
    //    Input (_pidInput) og Setpoint (_pidSetpoint) er sat
    _speedPID.Compute(); // Resultat ligger i _pidOutput

    // 6. Kombiner Feed-Forward og PID korrektion
    double finalPwm = (double)basePwm + _pidOutput;

    // 7. Begræns endelig PWM og send til motor
    finalPwm = constrain(finalPwm, 0, 255);
    _lastAppliedPwm = (int)finalPwm;
    _motor.applyRawPwm(_lastAppliedPwm);
}

// --- Stop ---
void SpeedController::stop() {
    _motor.applyRawPwm(0);
    _lastAppliedPwm = 0;
    // Skal PID resettes? Sæt evt. mode til MANUAL
    // _speedPID.SetMode(MANUAL);
}

// --- Getters (Eksempler) ---
double SpeedController::getActualRpm() const {
    return _pidInput; // Returner den (filtrerede) værdi PID'en bruger
}

int SpeedController::getLastPwm() const {
    return _lastAppliedPwm;
}

// --- Set Tunings ---
void SpeedController::setTunings(double Kp, double Ki, double Kd) {
    _kp = Kp;
    _ki = Ki;
    _kd = Kd;
    _speedPID.SetTunings(_kp, _ki, _kd);
}