// --------------- FIL: src/Motor.cpp ---------------
#include "Motor.h"
#include "config.h" // For PWM_MAX_DUTY, RPM_TIMEOUT_MS etc.
#include <Arduino.h>
#include <cmath>     // For round()

// --- Konstruktør ---
Motor::Motor(int pinIN1, int pinIN2, int pinENA, int hallPinA, int pwmChannel, int minMeasurementTimeMs) :
    _pinIN1(pinIN1),
    _pinIN2(pinIN2),
    _pinENA(pinENA),
    _hallPinA(hallPinA),
    _pwmChannel(pwmChannel),
    _pwmMax(PWM_MAX_DUTY),
    _minMeasurementTimeMs(minMeasurementTimeMs),
    _pulseCount(0),
    _actualRpm(0),
    _lastRpmUpdateTime(0),
    _startMeasurementTime(0),
    _currentDirectionForward(true)
{
}

// --- Initialisering ---
void Motor::begin() {
    pinMode(_pinIN1, OUTPUT);
    pinMode(_pinIN2, OUTPUT);
    double frequency = ledcSetup(_pwmChannel, PWM_FREQ, PWM_RESOLUTION);
    if (frequency == 0) {
        Serial.printf("[Motor %p] ERROR: ledcSetup failed for Chan %d\n", this, _pwmChannel);
    }
    ledcAttachPin(_pinENA, _pwmChannel);
    pinMode(_hallPinA, INPUT_PULLUP);
    setDirection(true);
    digitalWrite(_pinIN1, HIGH);
    digitalWrite(_pinIN2, LOW);
    applyRawPwm(0);
    resetPulseCount();
    _lastRpmUpdateTime = millis();
}

// --- Styring ---
void Motor::setDirection(bool forward) {
    if (forward != _currentDirectionForward || ledcRead(_pwmChannel) == 0 ) {
        digitalWrite(_pinIN1, forward ? HIGH : LOW);
        digitalWrite(_pinIN2, forward ? LOW : HIGH);
        _currentDirectionForward = forward;
    }
}

void Motor::stop() {
    applyRawPwm(0);
}

void Motor::applyRawPwm(int pwm) {
    pwm = constrain(pwm, 0, _pwmMax);
    ledcWrite(_pwmChannel, pwm);
}

// --- MÅLING (KORREKT, ROBUST VERSION) ---
int Motor::getActualRpm() {
    // 1. Tjek for timeout først. Hvis ingen pulser i lang tid, er RPM 0.
    if (_pulseCount == 0 && (millis() - _lastRpmUpdateTime > RPM_TIMEOUT_MS)) {
        _actualRpm = 0;
    }

    unsigned long currentTimeMicros = micros();
    unsigned long timePassedMicros = currentTimeMicros - _startMeasurementTime;

    // 2. Kør KUN beregningslogikken, hvis der er gået nok tid til en ny måling.
    if (timePassedMicros >= ((unsigned long)_minMeasurementTimeMs * 1000UL)) {
        
        // Læs og nulstil pulstælleren sikkert
        unsigned long currentPulseCountLocal = 0;
        noInterrupts();
        currentPulseCountLocal = _pulseCount;
        _pulseCount = 0;
        _startMeasurementTime = currentTimeMicros;
        interrupts();

        // Opdater timeout-timeren, da vi har forsøgt en måling
        _lastRpmUpdateTime = millis();

        // Beregn kun, hvis der rent faktisk var pulser
        if (currentPulseCountLocal > 0) {
            float pulsesPerSecond = (float)currentPulseCountLocal * 1000000.0f / timePassedMicros;
            float motorShaftRPM = (pulsesPerSecond * 60.0f) / COUNTS_PER_REV;
            int calculatedRpm = static_cast<int>(round(motorShaftRPM / GEAR_RATIO));

            // Sæt det korrekte fortegn
            if (!_currentDirectionForward) {
                calculatedRpm = -calculatedRpm;
            }
            _actualRpm = calculatedRpm;
        } else {
            // Hvis der ikke var nogen pulser i dette interval, er RPM 0 for dette interval
             _actualRpm = 0;
        }
    }

    // 3. Returner ALTID den senest kendte/beregnede værdi af _actualRpm.
    return _actualRpm;
}


// --- Interne Funktioner / ISR Hjælpere ---
void IRAM_ATTR Motor::incrementPulseCount() {
    _pulseCount++;
}

void Motor::resetPulseCount() {
    noInterrupts();
    _pulseCount = 0;
    _startMeasurementTime = micros();
    interrupts();
}