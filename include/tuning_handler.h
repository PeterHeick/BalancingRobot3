// tuning_handler.h
#ifndef TUNING_HANDLER_H
#define TUNING_HANDLER_H

#include <PID_v1.h> 

void initializeTuningParameters(); 
void saveTuningParameters();       
void handleSerialTuning(PID &pidController); 
void printCurrentTunings();        
// void processBufferedCommand(PID &pidController); // Kan fjernes herfra, hvis den kun er intern i .cpp
                                                // Men det skader ikke at have den.
#endif