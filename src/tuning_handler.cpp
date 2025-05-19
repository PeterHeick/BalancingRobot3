// tuning_handler.cpp
#include "tuning_handler.h" // Din header fil
#include "config.h"         // For globale extern deklarationer fra config.h
#include <Preferences.h>    // For NVS lagring
#include <Arduino.h>        // For Serial, String, byte, isprint, osv.
#include <stdint.h>         // For uint8_t

// Denne globale variabel skal være defineret i main.cpp (eller en anden .cpp fil)
// og deklareret som 'extern bool g_enable_csv_output;' i tuning_handler.h ELLER her.
// For at undgå linker fejl, hvis den også er i main.cpp, lad os antage den er i main.cpp
// og vi inkluderer den her via en extern deklaration, hvis den ikke er i tuning_handler.h
// Hvis g_enable_csv_output er defineret i main.cpp:
extern bool g_enable_csv_output;
// Hvis du vil have den defineret her i stedet for main.cpp:
// bool g_enable_csv_output = true; // Start med CSV output tændt

// ---- Globale Tuning Variable (Definitioner) ----
// Disse er standardværdierne, hvis intet er gemt i NVS, eller hvis NVS fejler.
double g_balance_kp = 3.3;                  // Start med en lav, sikker KP
double g_balance_ki = 0.0;                  // KI=0 indtil PD er stabil
double g_balance_kd = 0.20;                  // KD=0 indtil P er stærk nok til at oscillere
double g_init_balance = 0.8000;             // Din seneste værdi
double g_balance_output_to_rpm_scale = 1.0; // Start med neutral scale
double g_power_gain = 0.0;                 // Start med neutral scale

Preferences preferences; // Opret et globalt Preferences objekt for denne fil

// Nøgler til NVS
const char *PREF_NAMESPACE = "balancer"; // Namespace for dine indstillinger
const char *KEY_KP = "bal_kp";
const char *KEY_KI = "bal_ki";
const char *KEY_KD = "bal_kd";
const char *KEY_INIT_BAL = "init_bal";
const char *KEY_SCALE = "bal_scale";
const char *KEY_GAIN = "bal_gain";

// --- Buffer til seriel input ---
#define SERIAL_BUFFER_SIZE 64
char serialBuffer[SERIAL_BUFFER_SIZE]; // Buffer til at samle indkommende tegn
uint8_t bufferIndex = 0;               // Nuværende position i bufferen
bool commandReady = false;             // Flag der sættes, når en hel kommando er modtaget

// --- Funktionsprototyper for funktioner brugt internt i denne fil ---
void processBufferedCommand(PID &pidController); // Behandler den samlede kommando
// applyTuningsToPID er allerede deklareret i .h hvis den er public, ellers her hvis static/intern

// --- Funktionsdefinitioner ---

void initializeTuningParameters()
{
  bool nvs_opened_ok = preferences.begin(PREF_NAMESPACE, false); // false = read/write mode

  if (!nvs_opened_ok)
  {
    Serial.println("ADVARSEL: Kunne ikke åbne NVS/Preferences i load. Bruger default værdier.");
    // Lad g_... variablerne beholde deres kompileringstids-defaults
  }
  else
  {
    Serial.println("NVS/Preferences åbnet OK i load.");
    // Hent værdier fra NVS, eller brug de globale defaults hvis nøglen ikke findes
    g_balance_kp = preferences.getDouble(KEY_KP, g_balance_kp);
    g_balance_ki = preferences.getDouble(KEY_KI, g_balance_ki);
    g_balance_kd = preferences.getDouble(KEY_KD, g_balance_kd);
    g_init_balance = preferences.getDouble(KEY_INIT_BAL, g_init_balance);
    g_balance_output_to_rpm_scale = preferences.getDouble(KEY_SCALE, g_balance_output_to_rpm_scale);
    g_power_gain = preferences.getDouble(KEY_GAIN, g_power_gain);

    preferences.end(); // Luk preferences efter læsning
    Serial.println("Loaded tuning parameters from NVS (or defaults).");
  }
  printCurrentTunings();
}

void saveTuningParameters()
{
  if (!preferences.begin(PREF_NAMESPACE, false))
  { // Åbn for skrivning
    Serial.println("FEJL: Kunne ikke åbne preferences for skrivning i save!");
    return;
  }

  Serial.println("Attempting to save parameters to NVS...");
  preferences.putDouble(KEY_KP, g_balance_kp);
  preferences.putDouble(KEY_KI, g_balance_ki);
  preferences.putDouble(KEY_KD, g_balance_kd);
  preferences.putDouble(KEY_INIT_BAL, g_init_balance);
  preferences.putDouble(KEY_SCALE, g_balance_output_to_rpm_scale);
  preferences.putDouble(KEY_GAIN, g_power_gain);

  preferences.end(); // VIGTIGT: Luk for at sikre data skrives til flash!
  Serial.println("TAG_INFO: Tuning parameters saved to NVS.");
}

void applyTuningsToPID(PID &pidController)
{
  pidController.SetTunings(g_balance_kp, g_balance_ki, g_balance_kd);
}

void printCurrentTunings()
{
  Serial.printf("TAG_INFO: KP: %.4f, KI: %.4f, KD: %.4f, Pow: %.4f, InitBal: %.4f, Scale: %.4f, CSV: %s\n",
                g_balance_kp, g_balance_ki, g_balance_kd, g_power_gain, g_init_balance, g_balance_output_to_rpm_scale,
                g_enable_csv_output ? "ON" : "OFF");
}

void handleSerialTuning(PID &pidController)
{
  while (Serial.available() > 0 && !commandReady)
  {
    char receivedChar = Serial.read();

    if (receivedChar == '\n' || receivedChar == '\r')
    { // Newline eller carriage return
      if (bufferIndex > 0)
      {                                   // Hvis der er noget i bufferen
        serialBuffer[bufferIndex] = '\0'; // Nul-terminer strengen
        commandReady = true;
      }
      else
      {
        // Modtaget newline/cr på en tom buffer, bufferIndex er allerede 0 eller nulstilles nedenfor.
      }
    }
    else if (isprint(receivedChar))
    { // Kun printbare tegn
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1)
      {
        serialBuffer[bufferIndex++] = receivedChar;
      }
      else
      {
        // Buffer overflow
        Serial.println("Error: Command too long. Buffer flushed.");
        bufferIndex = 0; // Nulstil/flush buffer
        // Ryd resten af den potentielt lange linje fra seriel input buffer
        while (Serial.available() > 0 && Serial.read() != '\n')
          ;
      }
    }
    // Ignorer andre ikke-printbare tegn
  }

  if (commandReady)
  {
    processBufferedCommand(pidController);
    commandReady = false; // Nulstil flaget efter behandling
    bufferIndex = 0;      // Nulstil buffer index for næste kommando
  }
}

void processBufferedCommand(PID &pidController)
{
  String input(serialBuffer); // Konverter char array (serialBuffer) til et String objekt
  input.trim();

  // Debug print for at se hvad der processeres
  Serial.print("DBG PROC: Processing '"); Serial.print(input); Serial.println("'");

  if (input.equalsIgnoreCase("save"))
  {
    saveTuningParameters();
  }
  else if (input.equalsIgnoreCase("load"))
  {
    initializeTuningParameters();
  }
  else if (input.equalsIgnoreCase("print"))
  {
    printCurrentTunings();
  }
  else if (input.equalsIgnoreCase("csv_on"))
  {
    g_enable_csv_output = true;
    Serial.println("CSV output ENABLED.");
    printCurrentTunings();
  }
  else if (input.equalsIgnoreCase("csv_off"))
  {
    g_enable_csv_output = false;
    Serial.println("CSV output DISABLED.");
    printCurrentTunings();
  }
  else if (input.equalsIgnoreCase("help"))
  {
    Serial.println("TAG_INFO: Available commands (case-insensitive for command part):");
    Serial.println("TAG_INFO:   kp=<value>    (e.g., kp=2.5)");
    Serial.println("TAG_INFO:   ki=<value>    (e.g., ki=0.01)");
    Serial.println("TAG_INFO:   kd=<value>    (e.g., kd=0.15)");
    Serial.println("TAG_INFO:   gain=<value>   (e.g., gain=0.01)");
    Serial.println("TAG_INFO:   init=<value>  (e.g., init=0.5)");
    Serial.println("TAG_INFO:   scale=<value> (e.g., scale=1.2)");
    Serial.println("TAG_INFO:   save          (saves current tunings to NVS)");
    Serial.println("TAG_INFO:   print         (prints current tunings)");
    Serial.println("TAG_INFO:   csv_on        (enables CSV data output)");
    Serial.println("TAG_INFO:   csv_off       (disables CSV data output)");
    Serial.println("TAG_INFO:   help          (shows this help message)");
  }
  else
  {
    int equalsPos = input.indexOf('=');
    // Sørg for at der er et tegn før '=' og mindst et tegn efter
    if (equalsPos > 0 && equalsPos < input.length() - 1)
    {
      String command = input.substring(0, equalsPos);
      command.toLowerCase(); // Gør kommando-delen case-insensitive
      String valueStr = input.substring(equalsPos + 1);
      double value = valueStr.toDouble();

      bool pid_params_changed = false;

      if (command.equals("kp"))
      {
        g_balance_kp = value;
        pid_params_changed = true;
      }
      else if (command.equals("ki"))
      {
        g_balance_ki = value;
        pid_params_changed = true;
      }
      else if (command.equals("kd"))
      {
        g_balance_kd = value;
        pid_params_changed = true;
      }
      else if (command.equals("pow") || command.equals("gain"))
      {
        g_power_gain = value;
        pid_params_changed = true;
      }
      else if (command.equals("init"))
      {
        g_init_balance = value;
        Serial.printf("INITBALANCE set to: %.4f\n", g_init_balance);
      }
      else if (command.equals("scale"))
      {
        g_balance_output_to_rpm_scale = value;
        Serial.printf("SCALE set to: %.4f\n", g_balance_output_to_rpm_scale);
      }
      else
      {
        Serial.print("Unknown command parameter: '");
        Serial.print(command);
        Serial.println("'");
      }

      if (pid_params_changed)
      {
        applyTuningsToPID(pidController);
        Serial.printf("Updated PID param %s to: %.4f\n", command.c_str(), value);
      }

      // Print altid nuværende tunings efter en gyldig parameterændring eller 'init'/'scale'
      if (pid_params_changed || command.equals("init") || command.equals("scale"))
      {
        printCurrentTunings();
      }
    }
    else if (input.length() > 0)
    { // Hvis der var input, men ikke en gyldig kommando=værdi
      Serial.print("TAG_INFO: Invalid command or format: '");
      Serial.print(input);
      Serial.println("'. Type 'help'.");
    }
    // Hvis input.length() == 0 (f.eks. kun newline sendt og bufferen var tom), sker der intet her.
  }
}