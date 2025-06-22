#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <Arduino.h> // Til Serial
#include "config.h"  // Til g_enable_csv_output

class CsvLogger {
public:
    // Initialisering (ikke meget at gøre her, men god praksis)
    void begin() {
        // Kan evt. tjekke om Serial er initialiseret, men antager setup() gør det
    }

    // Printer CSV header linjen. Skal kaldes en gang i setup().
    void printHeader();

    // Logger en række data. Kaldes hver loop iteration.
    // nowMicros: Aktuel tid i mikrosekunder
    // fusedPitch: Aktuel pitch vinkel (grader)
    // fusedPitchRate: Aktuel pitch rate (grader/sek). Kan være rå eller filtreret.
    // balanceCmd: Rå kontrol output fra BalanceController (efter constrain, før scaling/gain)
    // pTerm, iTerm, dTerm: De individuelle PID termer fra BalanceController
    // scaledOutput: Det endelige scaled output sendt til motorerne
    void logData(unsigned long nowMicros,
                 double fusedPitch,
                 double fusedPitchRate,
                 double balanceCmd,
                 double pTerm,
                 double iTerm,
                 double dTerm,
                 double scaledOutput,
                 double totalDistance_m);
};

#endif // CSV_LOGGER_H