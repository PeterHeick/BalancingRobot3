#ifndef TUNING_HANDLER_H
#define TUNING_HANDLER_H

// Vi behøver ikke PID_v1 headeren her længere
// #include <PID_v1.h> // <--- FJERNES

// Deklarerer de globale tuning variabler. De defineres i tuning_handler.cpp.
extern double g_balance_kp;
extern double g_balance_ki;
extern double g_balance_kd;
extern double g_init_balance;
extern double g_balance_output_to_rpm_scale;
extern double g_power_gain;

// Deklarer den globale variabel til CSV output, defineret i tuning_handler.cpp
extern bool g_enable_csv_output;

// Funktioner til håndtering af tuning
void initializeTuningParameters(); // Læser fra NVS eller sætter defaults
void saveTuningParameters();       // Gemmer til NVS
void printCurrentTunings();        // Printer nuværende værdier

// Funktion til at håndtere seriel input og parse kommandoer.
// Modtager nu aktuel pitch for 'init_now' kommandoen.
void handleSerialTuning(double currentPitch); // <-- RETTET SIGANTUR

// Intern funktion (prototypen kan fjernes hvis den kun bruges i .cpp, men ok at have)
// Modtager nu aktuel pitch
void processBufferedCommand(double currentPitch); // <-- RETTET SIGANTUR

#endif // TUNING_HANDLER_H