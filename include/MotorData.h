#ifndef MOTORDATA_H
#define MOTORDATA_H

#include "config.h" // For RPM_LOOKUP_TABLE_SIZE

// Deklarerer at disse arrays findes et andet sted
extern const int RPM_TO_PWM_MOTOR_1[RPM_LOOKUP_TABLE_SIZE];
extern const int RPM_TO_PWM_MOTOR_2[RPM_LOOKUP_TABLE_SIZE];

#endif