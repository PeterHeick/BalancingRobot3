// tuning_handler.cpp
#include "tuning_handler.h"
#include "config.h"
#include <Preferences.h>
#include <Arduino.h>
#include <stdint.h>

// ---- Globale Tuning Variable (Definitioner) ----
// Disse defineres her, og deklareres 'extern' i config.h og tuning_handler.h
double g_balance_kp = 18.0;
double g_balance_ki = 0.1;
double g_balance_kd = 0.2;
double g_velocity_kp = 0.0;
double g_init_balance = -2.5000;
double g_balance_output_to_rpm_scale = 1.0;
double g_power_gain = 0.0;
double g_balance_calib_ki = 0.001;

bool g_enable_csv_output = false;

Preferences preferences;

const char *PREF_NAMESPACE = "balancer";
const char *KEY_KP = "bal_kp";
const char *KEY_KI = "bal_ki";
const char *KEY_KD = "bal_kd";
const char *KEY_VEL_KP = "vel_kp";
const char *KEY_INIT_BAL = "init_bal";
const char *KEY_SCALE = "bal_scale";
const char *KEY_GAIN = "bal_gain";
const char *KEY_CALIB_KI = "calib_ki";


// --- Buffer til seriel input ---
#define SERIAL_BUFFER_SIZE 64
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t bufferIndex = 0;
bool commandReady = false;

// Prototype for internal use only (can be static)
static void processBufferedCommand();

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
    g_velocity_kp = preferences.getDouble(KEY_VEL_KP, g_velocity_kp); // <--- LOAD
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
  preferences.putDouble(KEY_VEL_KP, g_velocity_kp); // <--- SAVE
  preferences.putDouble(KEY_INIT_BAL, g_init_balance);
  preferences.putDouble(KEY_SCALE, g_balance_output_to_rpm_scale);
  preferences.putDouble(KEY_GAIN, g_power_gain);

  preferences.end();
  Serial.println("TAG_INFO: Tuning parameters saved to NVS.");
}

void printCurrentTunings()
{
  // Print g_velocity_kp
  Serial.printf("TAG_INFO: KP: %.4f, KI: %.4f, KD: %.4f, VelKP: %.4f, Gain: %.4f, InitBal: %.4f, Scale: %.4f, CSV: %s\n",
                g_balance_kp, g_balance_ki, g_balance_kd, g_velocity_kp, g_power_gain, g_init_balance, g_balance_output_to_rpm_scale,
                g_enable_csv_output ? "ON" : "OFF");
}

void handleSerialTuning()
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
        while (Serial.available() > 0 && Serial.read() != '\n'); // Clear rest of line
      }
    }
  }

  if (commandReady)
  {
    processBufferedCommand();
    commandReady = false;
    bufferIndex = 0;
  }
}

static void processBufferedCommand() // Made static
{
  String input(serialBuffer);
  input.trim();

  if (input.equalsIgnoreCase("save"))
  {
    saveTuningParameters();
  }
  else if (input.equalsIgnoreCase("load"))
  {
    Serial.println("TAG_INFO: Loading parameters from NVS...");
    initializeTuningParameters();
  }
  else if (input.equalsIgnoreCase("print"))
  {
    printCurrentTunings();
  }
  else if (input.equalsIgnoreCase("csv_on"))
  {
    g_enable_csv_output = true;
    netDisplacement_m = 0.0;
    Serial.println("TAG_INFO: CSV output ENABLED.");
  }
  else if (input.equalsIgnoreCase("csv_off"))
  {
    g_enable_csv_output = false;
    Serial.println("TAG_INFO: CSV output DISABLED.");
  }
  else if (input.equalsIgnoreCase("help"))
  {
    Serial.println("TAG_INFO: Available commands (case-insensitive for command part):");
    Serial.println("TAG_INFO:   kp=<value>");
    Serial.println("TAG_INFO:   ki=<value>");
    Serial.println("TAG_INFO:   kd=<value>");
    Serial.println("TAG_INFO:   velkp=<value>"); // <--- HELP MESSAGE
    Serial.println("TAG_INFO:   gain=<value>");
    Serial.println("TAG_INFO:   init=<value>");
    // Serial.println("TAG_INFO:   init_now");
    Serial.println("TAG_INFO:   scale=<value>");
    Serial.println("TAG_INFO:   save");
    Serial.println("TAG_INFO:   load");
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
       else if (command.equals("velkp"))
      {
        g_velocity_kp = value;
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
    }
    else if (input.length() > 0)
    {
      Serial.print("TAG_INFO: Invalid command or format: '");
      Serial.print(input);
      Serial.println("'. Type 'help'.");
    }
  }
}