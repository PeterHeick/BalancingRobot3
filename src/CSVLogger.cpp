#include "CSVLogger.h"
#include "config.h" // Inkluder config for g_enable_csv_output

// --- Print Header ---
void CsvLogger::printHeader()
{
  if (g_enable_csv_output)
  {
    // CSV format: tid_ms,fusedPitch,fusedPitchRate,balanceCmd,pTerm,iTerm,dTerm,scaledOutput
    Serial.println("TAG_CSV_HEADER: time_ms,fusedPitch,fusedPitchRate,balanceCmd,pTerm,iTerm,dTerm,scaledOutput,totalDistance_m");
  }
}

// --- Log Data ---
void CsvLogger::logData(unsigned long nowMicros,
                        double fusedPitch,
                        double fusedPitchRate,
                        double balanceCmd,
                        double pTerm,
                        double iTerm,
                        double dTerm,
                        double scaledOutput,
                        double totalDistance_m)
{
  if (g_enable_csv_output)
  {
    Serial.printf("TAG_CSV: %.4f,", nowMicros / 1000.0); // Tid i ms
    Serial.printf("%.4f,", fusedPitch);                  // Fused Pitch (Grader)
    Serial.printf("%.4f,", fusedPitchRate);              // Fused Pitch Rate (Grader/sek)
    Serial.printf("%.4f,", balanceCmd);                  // Rå Kontrol Output (EFTER constrain)
    Serial.printf("%.4f,", pTerm);                       // P-term bidrag
    Serial.printf("%.4f,", iTerm);                       // I-term bidrag
    Serial.printf("%.4f,", dTerm);                       // D-term bidrag
    Serial.printf("%.4f,", scaledOutput);   
    Serial.printf("%.4f\n", totalDistance_m);           // Akkumuleret distance i meter (TILFØJET)
  }
}