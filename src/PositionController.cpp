#include "PositionController.h"
#include "config.h"

PositionController::PositionController(Motor* motor1, Motor* motor2) :
    _motor1(motor1),
    _motor2(motor2),
    _totalPulses1(0),
    _totalPulses2(0),
    _lastPulses1(0),
    _lastPulses2(0),
    _currentPosition(0.0),
    _positionSetpoint(0.0),
    _currentVelocity(0.0),
    _lastVelocityUpdate(0),
    _lastPosition(0.0),
    _lastError(0.0),
    _integral(0.0),
    _lastTime(0)
{
}

void PositionController::begin() {
    resetPosition();
    _lastTime = millis();
    _lastVelocityUpdate = millis();
}

void PositionController::resetPosition() {
    noInterrupts();
    _totalPulses1 = 0;
    _totalPulses2 = 0;
    _lastPulses1 = 0;
    _lastPulses2 = 0;
    interrupts();
    
    _currentPosition = 0.0;
    _positionSetpoint = 0.0;
    _currentVelocity = 0.0;
    _lastPosition = 0.0;
    _integral = 0.0;
    _lastError = 0.0;
}

double PositionController::pulsesToDistance(long pulses) {
    // Beregn distance i meter fra encoder pulser
    // pulser -> hjul omdrejninger -> lineær distance
    double wheelRevolutions = (double)pulses / (COUNTS_PER_REV * GEAR_RATIO);
    return wheelRevolutions * WHEEL_DIAMETER_M * PI;
}

void PositionController::updateMotorPulses() {
    // Simpel implementering der bruger RPM til at estimere position
    // Ikke perfekt, men et startpunkt for at teste position kontrollen
    
    unsigned long currentTime = millis();
    static unsigned long lastUpdateTime = currentTime;
    
    if (currentTime > lastUpdateTime) {
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // sekunder
        
        // Få RPM fra motorer
        double rpm1 = _motor1->getActualRpm();
        double rpm2 = _motor2->getActualRpm();
        
        // Beregn rotation i denne periode (omdrejninger)
        double rotations1 = (rpm1 / 60.0) * deltaTime;
        double rotations2 = (rpm2 / 60.0) * deltaTime;
        
        // Konverter til pulser og akkumuler
        long deltasPulses1 = (long)(rotations1 * COUNTS_PER_REV * GEAR_RATIO);
        long deltasPulses2 = (long)(rotations2 * COUNTS_PER_REV * GEAR_RATIO);
        
        _totalPulses1 += deltasPulses1;
        _totalPulses2 += deltasPulses2;
        
        lastUpdateTime = currentTime;
    }
}

void PositionController::updateVelocity() {
    unsigned long currentTime = millis();
    if (currentTime - _lastVelocityUpdate >= 50) { // Opdater hver 50ms
        double deltaTime = (currentTime - _lastVelocityUpdate) / 1000.0; // sekunder
        _currentVelocity = (_currentPosition - _lastPosition) / deltaTime;
        
        _lastPosition = _currentPosition;
        _lastVelocityUpdate = currentTime;
    }
}

void PositionController::update() {
    updateMotorPulses();
    
    // Beregn gennemsnitlig position fra begge motorer
    long avgPulses = (_totalPulses1 + _totalPulses2) / 2;
    _currentPosition = pulsesToDistance(avgPulses);
    
    updateVelocity();
}

double PositionController::computePositionPID() {
    unsigned long currentTime = millis();
    double deltaTime = (currentTime - _lastTime) / 1000.0; // sekunder
    
    if (deltaTime <= 0.0) return 0.0;
    
    // Beregn fejl
    double error = _positionSetpoint - _currentPosition;
    
    // Proportional term
    double proportional = g_position_kp * error;
    
    // Integral term (med anti-windup)
    if (abs(error) < POSITION_ANTI_WINDUP_THRESHOLD) {
        _integral += error * deltaTime;
        _integral = constrain(_integral, -POSITION_INTEGRAL_LIMIT, POSITION_INTEGRAL_LIMIT);
    }
    double integral = g_position_ki * _integral;
    
    // Derivative term
    double derivative = g_position_kd * (error - _lastError) / deltaTime;
    
    // Samlet PID output
    double output = proportional + integral + derivative;
    output = constrain(output, -POSITION_PID_OUTPUT_LIMIT, POSITION_PID_OUTPUT_LIMIT);
    
    // Gem værdier til næste iteration
    _lastError = error;
    _lastTime = currentTime;
    
    return output;
}