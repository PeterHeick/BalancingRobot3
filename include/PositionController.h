#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <Arduino.h>
#include "Motor.h"
#include "config.h"

#define WHEEL_DIAMETER_M 0.120  // Hjuldiameter i meter (67mm)
// PI er allerede defineret i Arduino.h

class PositionController {
private:
    Motor* _motor1;
    Motor* _motor2;
    
    // Position tracking
    volatile long _totalPulses1;
    volatile long _totalPulses2;
    volatile long _lastPulses1;
    volatile long _lastPulses2;
    
    double _currentPosition;
    double _positionSetpoint;
    
    // Velocity calculation
    double _currentVelocity;
    unsigned long _lastVelocityUpdate;
    double _lastPosition;
    
    // PID variables
    double _lastError;
    double _integral;
    double _lastTime;
    
    // Position calculations
    double pulsesToDistance(long pulses);
    void updateVelocity();
    
public:
    PositionController(Motor* motor1, Motor* motor2);
    
    void begin();
    void update();
    void resetPosition();
    
    // Position getters
    double getCurrentPosition() { return _currentPosition; }
    double getCurrentVelocity() { return _currentVelocity; }
    double getPositionSetpoint() { return _positionSetpoint; }
    
    // Position setters
    void setPositionSetpoint(double setpoint) { _positionSetpoint = setpoint; }
    
    // PID computation
    double computePositionPID();
    
    // Motor pulse tracking
    void updateMotorPulses();
};

#endif // POSITION_CONTROLLER_H