#include "SpeedController.h"
#include <Arduino.h> // For constrain, abs, osv.
#include "config.h"  // For diverse konstanter

// --- Konstruktør (SIMPLIFICERET) ---
SpeedController::SpeedController(Motor &motor) :
    _motor(motor), // Gem reference til motor
    _lastAppliedPwm(0)
{
    // Ingen PID eller feedforward tables længere!
}

// --- Initialisering (SIMPLIFICERET) ---
void SpeedController::begin() {
    // Ingen PID initialisering - kun motor setup hvis nødvendig
    _lastAppliedPwm = 0;
    _motor.applyRawPwm(0); // Start med stop
}

// --- Sæt Direkte PWM Baseret på Target RPM (NY TILGANG) ---
void SpeedController::setTargetRpm(double targetRpm) {
    // Konverter RPM direkte til PWM uden feedforward eller PID
    
    // Håndter retning
    bool forward = targetRpm >= 0;
    _motor.setDirection(forward);
    
    // Konverter RPM til PWM med simpel skalering
    double absRpm = abs(targetRpm);
    
    // Deadzone - under denne RPM sender vi 0 PWM
    if (absRpm < DEADZONE_RPM_THRESHOLD) {
        _lastAppliedPwm = 0;
        _motor.applyRawPwm(0);
        return;
    }
    
    // Simpel lineær skalering: RPM -> PWM
    // Du skal justere disse konstanter baseret på dine motore
    // Eksempel: 0-100 RPM maps til 50-255 PWM (minimum PWM at starte)
    
    int pwm;
    if (absRpm <= MIN_EFFECTIVE_RPM) {
        // Under minimum RPM - brug minimum PWM
        pwm = MIN_MOTOR_PWM;
    } else if (absRpm >= MAX_RPM) {
        // Over maximum RPM - brug maximum PWM  
        pwm = 255;
    } else {
        // Lineær interpolation mellem MIN og MAX
        // pwm = MIN_PWM + (RPM - MIN_RPM) * (MAX_PWM - MIN_PWM) / (MAX_RPM - MIN_RPM)
        pwm = MIN_MOTOR_PWM + 
              (absRpm - MIN_EFFECTIVE_RPM) * (255 - MIN_MOTOR_PWM) / 
              (MAX_RPM - MIN_EFFECTIVE_RPM);
    }
    
    // Begræns PWM
    pwm = constrain(pwm, 0, 255);
    
    // Send til motor
    _lastAppliedPwm = pwm;
    _motor.applyRawPwm(pwm);
}

// --- Update er nu meget simpel ---
void SpeedController::update(int fortegn) {
    // Ingen update nødvendig - alt sker i setTargetRpm
    // Denne metode kan være tom eller fjernes helt
    
    // Optionelt: Du kan læse actual RPM for debugging
    // double actualRpm = _motor.getActualRpm();
    // Serial.print("Target PWM: "); Serial.print(_lastAppliedPwm);
    // Serial.print(", Actual RPM: "); Serial.println(actualRpm);
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
void SpeedController::setTunings(double Kp, double Ki, double Kd) {
    // Ingen PID længere - denne metode gør intet
    // Kan fjernes fra header fil også
}