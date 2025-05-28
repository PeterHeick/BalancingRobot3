// tuning_handler.cpp
#include "tuning_handler.h"
#include "config.h"
#include <Preferences.h>
#include <Arduino.h>
#include <stdint.h>

// ---- Globale Tuning Variable (Definitioner) ----
double g_balance_kp = 3.3;
double g_balance_ki = 0.0;
double g_balance_kd = 0.20;
double g_init_balance = 0.8000;
double g_balance_output_to_rpm_scale = 1.0;
double g_power_gain = 0.0;

bool g_enable_csv_output = false;

Preferences preferences;

const char *PREF_NAMESPACE = "balancer";
const char *KEY_KP = "bal_kp";
const char *KEY_KI = "bal_ki";
const char *KEY_KD = "bal_kd";
const char *KEY_INIT_BAL = "init_bal";
const char *KEY_SCALE = "bal_scale";
const char *KEY_GAIN = "bal_gain";

// --- Buffer til seriel input ---
#define SERIAL_BUFFER_SIZE 64
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t bufferIndex = 0;
bool commandReady = false;

void processBufferedCommand(double currentPitch); // Prototype

void initializeTuningParameters()
{
  bool nvs_opened_ok = preferences.begin(PREF_NAMESPACE, false);

  if (!nvs_opened_ok)
  {
    Serial.println("ADVARSEL: Kunne ikke åbne NVS/Preferences i load. Bruger default værdier.");
  }
  else
  {
    Serial.println("NVS/Preferences åbnet OK i load.");
    g_balance_kp = preferences.getDouble(KEY_KP, g_balance_kp);
    g_balance_ki = preferences.getDouble(KEY_KI, g_balance_ki);
    g_balance_kd = preferences.getDouble(KEY_KD, g_balance_kd);
    g_init_balance = preferences.getDouble(KEY_INIT_BAL, g_init_balance);
    g_balance_output_to_rpm_scale = preferences.getDouble(KEY_SCALE, g_balance_output_to_rpm_scale);
    g_power_gain = preferences.getDouble(KEY_GAIN, g_power_gain);

    preferences.end();
    Serial.println("Loaded tuning parameters from NVS (or defaults).");
  }
  printCurrentTunings(); // Print værdierne ved opstart
}

void saveTuningParameters()
{
  if (!preferences.begin(PREF_NAMESPACE, false))
  {
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

  preferences.end();
  Serial.println("TAG_INFO: Tuning parameters saved to NVS.");
}

void printCurrentTunings()
{
  // THIS IS THE FORMAT YOUR PYTHON SCRIPT EXPECTS FOR VERIFICATION
  // Do NOT add extra newlines or text around this specific print format.
  Serial.printf("TAG_INFO: KP: %.4f, KI: %.4f, KD: %.4f, Gain: %.4f, InitBal: %.4f, Scale: %.4f, CSV: %s\n",
                g_balance_kp, g_balance_ki, g_balance_kd, g_power_gain, g_init_balance, g_balance_output_to_rpm_scale,
                g_enable_csv_output ? "ON" : "OFF");
}

void handleSerialTuning(double currentPitch)
{
  while (Serial.available() > 0 && !commandReady)
  {
    char receivedChar = Serial.read();

    if (receivedChar == '\n' || receivedChar == '\r')
    {
      if (bufferIndex > 0)
      {
        serialBuffer[bufferIndex] = '\0';
        commandReady = true;
      }
    }
    else if (isprint(receivedChar))
    {
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1)
      {
        serialBuffer[bufferIndex++] = receivedChar;
      }
      else
      {
        Serial.println("Error: Command too long. Buffer flushed.");
        bufferIndex = 0;
        while (Serial.available() > 0 && Serial.read() != '\n');
      }
    }
  }

  if (commandReady)
  {
    processBufferedCommand(currentPitch);
    commandReady = false;
    bufferIndex = 0;
  }
}

void processBufferedCommand(double currentPitch)
{
  String input(serialBuffer);
  input.trim();

  // Debug print kan være nyttigt, men kan forstyrre Python parseren.
  // Hold den kommenteret ud medmindre du specifikt debugger her.
  // Serial.print("DBG PROC: Processing '"); Serial.print(input); Serial.println("'");

  if (input.equalsIgnoreCase("save"))
  {
    saveTuningParameters();
  }
  else if (input.equalsIgnoreCase("load"))
    // Load from NVS, but do NOT print immediately here.
    // Python sends 'print' afterwards.
  {
    Serial.println("TAG_INFO: Loading parameters from NVS..."); // Info message BEFORE loading
    // preferences.begin / getDouble / preferences.end are called inside initializeTuningParameters()
    initializeTuningParameters(); // This also prints at the end of its execution
  }
  else if (input.equalsIgnoreCase("print"))
  {
    // This is the command Python uses to verify parameters.
    // ONLY call printCurrentTunings() here.
    printCurrentTunings();
  }
  else if (input.equalsIgnoreCase("csv_on"))
  {
    g_enable_csv_output = true;
    Serial.println("TAG_INFO: CSV output ENABLED.");
    // Do NOT print tunings here automatically, Python sends print after.
  }
  else if (input.equalsIgnoreCase("csv_off"))
  {
    g_enable_csv_output = false;
    Serial.println("TAG_INFO: CSV output DISABLED.");
     // Do NOT print tunings here automatically, Python sends print after.
  }
   else if (input.equalsIgnoreCase("init_now")) {
    g_init_balance = currentPitch;
    Serial.printf("TAG_INFO: Initial balance angle set to current pitch: %.4f\n", g_init_balance);
    // Do NOT print tunings here automatically, Python sends print after.
   }
  else if (input.equalsIgnoreCase("help"))
  {
    Serial.println("TAG_INFO: Available commands (case-insensitive for command part):");
    Serial.println("TAG_INFO:   kp=<value>");
    Serial.println("TAG_INFO:   ki=<value>");
    Serial.println("TAG_INFO:   kd=<value>");
    Serial.println("TAG_INFO:   gain=<value>");
    Serial.println("TAG_INFO:   init=<value>");
    Serial.println("TAG_INFO:   init_now");
    Serial.println("TAG_INFO:   scale=<value>");
    Serial.println("TAG_INFO:   save");
    Serial.println("TAG_INFO:   load"); // Added load command help
    Serial.println("TAG_INFO:   print");
    Serial.println("TAG_INFO:   csv_on");
    Serial.println("TAG_INFO:   csv_off");
    Serial.println("TAG_INFO:   help");
  }
  else
  {
    int equalsPos = input.indexOf('=');
    if (equalsPos > 0 && equalsPos < input.length() - 1)
    {
      String command = input.substring(0, equalsPos);
      command.toLowerCase();
      String valueStr = input.substring(equalsPos + 1);
      double value = valueStr.toDouble();

      // Update the global variable directly
      if (command.equals("kp"))
      {
        g_balance_kp = value;
      }
      else if (command.equals("ki"))
      {
        g_balance_ki = value;
      }
      else if (command.equals("kd"))
      {
        g_balance_kd = value;
      }
      else if (command.equals("gain"))
      {
        g_power_gain = value;
      }
      else if (command.equals("init"))
      {
        g_init_balance = value;
      }
      else if (command.equals("scale"))
      {
        g_balance_output_to_rpm_scale = value;
      }
      else
      {
        Serial.print("TAG_INFO: Unknown command parameter: '");
        Serial.print(command);
        Serial.println("'");
      }
      // Do NOT print status here after each parameter update.
      // Python expects to control when the robot prints status via the 'print' command.
    }
    else if (input.length() > 0)
    {
      Serial.print("TAG_INFO: Invalid command or format: '");
      Serial.print(input);
      Serial.println("'. Type 'help'.");
    }
  }
}