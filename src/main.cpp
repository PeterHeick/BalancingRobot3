/**
 * main.cpp
 * Hovedprogram for Balancerobot - Main Logic.
 */

#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include "config.h"            // Konfigurationskonstanter (inkluderer extern deklarationer for g_ og RobotState enum)
#include "ESP32.h"             // Pin definitioner (Nu uden konflikterende standardnavne)
#include "Motor.h"             // Grundlæggende Motor klasse
#include "SpeedController.h"   // Hastighedsregulator klasse
#include "BalanceController.h" // BalanceController klasse
#include "IMUManager.h"        // <--- Inkluder den nye IMU Manager klasse
#include "CSVLogger.h"         // <--- Inkluder den nye CSV Logger klasse
#include "tuning_handler.h"    // Håndterer seriel tuning og globale tuning variabler

// --- Globale Objekter ---
Motor motor1(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENA, MOTOR1_HALL_A, PWM_CHANNEL1, MOTOR_MIN_MEASUREMENT_TIME_MS);
Motor motor2(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_ENB, MOTOR2_HALL_A, PWM_CHANNEL2, MOTOR_MIN_MEASUREMENT_TIME_MS);

SpeedController speedCtrl1(motor1);
SpeedController speedCtrl2(motor2);

BalanceController balanceController; // BalanceController objekt
IMUManager imuManager;               // <--- IMU Manager objekt
CsvLogger csvLogger;                 // <--- CSV Logger objekt

// --- Globale Tuning Variable ---
// Deres definitioner er i tuning_handler.cpp, deklarationer i config.h
// g_balance_kp, g_balance_ki, g_balance_kd, g_velocity_kp, g_init_balance, g_balance_output_to_rpm_scale, g_power_gain, g_enable_csv_output

// --- Globale Tilstandsvariabler ---
// RobotState enum er defineret i config.h
RobotState currentState = IDLE;
const char *RobotStateString[] = {"IDLE", "CALIBRATING_IMU", "BALANCING", "FALLEN"};

extern double g_balance_calib_ki;
unsigned long lastLoopTimeMicros = 0;
double netDisplacement_m = 0.0;
double nvs_g_init_balance = 0.0;        // Gemmer den oprindelige g_init_balance fra NVS
unsigned long balancing_start_time = 0; // Timer for hvor længe vi har balanceret
bool has_saved_this_session = false;

// NYE variabler til stabilitets-tjek
unsigned long stability_timer_start = 0;

// --- ISR Funktioner ---
void IRAM_ATTR motor1_isrA() { motor1.incrementPulseCount(); }
void IRAM_ATTR motor2_isrA() { motor2.incrementPulseCount(); }

// --- Setup ---
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("\n\n--- Balancerobot V6 ---");
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());

  initializeTuningParameters(); // Hent/initialiser tuning værdier fra NVS
  nvs_g_init_balance = g_init_balance;

  // Initialiser hardware
  Serial.println("Initialiserer motorer...");
  motor1.begin();
  motor2.begin();
  Serial.println("Motorer OK.");

  // Initialiser IMU Manager - den initialiserer BNO085 internt
  if (!imuManager.begin())
  {
    // imuManager.begin() printer selv fejlmeddelelser
    Serial.println("Fatal fejl: IMU initialisering fejlede.");
    while (1)
      ; // Stop her hvis IMU ikke virker
  }
  Serial.println("IMU Manager OK.");

  Serial.println("Initialiserer Speed Controllers...");
  speedCtrl1.begin();
  speedCtrl2.begin();
  Serial.println("Speed Controllers OK.");

  // Initialiser Balance Controller
  balanceController.begin();
  Serial.println("Balance Controller OK.");

  // Initialiser CSV Logger
  csvLogger.begin();
  Serial.println("CSV Logger OK.");

  Serial.println("Tilknytter Interrupts...");
  attachInterrupt(digitalPinToInterrupt(MOTOR1_HALL_A), motor1_isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_HALL_A), motor2_isrA, RISING);
  Serial.println("Interrupts OK.");

  lastLoopTimeMicros = micros();

  // Vent kort og start BALANCING state
  delay(2000);

  // Sæt den initiale balance vinkel baseret på nuværende pitch
  // Læs IMU en gang for at få en startværdi til init_balance
  // Dette er en lidt simplere version af setup, da IMUManager håndterer selve læsningen
  // Vi skal bare kalde update og derefter getPitch.
  unsigned long start_time = millis();
  double startupPitch = g_init_balance; // Start med NVS/default
  bool pitch_available = false;

  // Prøv at læse IMU reports et par gange for at få en første pitch værdi
  for (int i = 0; i < IMU_STARTUP_READ_ATTEMPTS; ++i)
  {                                       // Prøv op til 10 gange over 500ms (10*50ms)
    imuManager.update();                  // Læs reports
    startupPitch = imuManager.getPitch(); // Hent den seneste pitch
    if (abs(startupPitch) > 0.001)
    { // Antag en non-zero pitch betyder vi har data
      pitch_available = true;
      break;
    }
    delay(IMU_STARTUP_READ_DELAY_MS);
  }

  if (pitch_available)
  {
    g_init_balance = startupPitch;
    Serial.printf("TAG_INFO: Initial balance angle set to current pitch after setup: %.4f\n", g_init_balance);
  }
  else
  {
    Serial.println("ADVARSEL: Kunne ikke få start pitch fra BNO085 indenfor timeout. Bruger NVS/default g_init_balance.");
  }
  printCurrentTunings();

  // Print CSV header - nu via logger objektet
  csvLogger.printHeader();

  currentState = BALANCING; // Nu klar til at starte balancing
  Serial.println("TAG_STATE: Entering BALANCING state.");
}

// --- Loop ---
void loop()
{
  handleSerialTuning();

  unsigned long nowMicros = micros();
  unsigned long stability_timer_start = 0;
  unsigned long prt_timer_start = 0;
  double pid_dt = (double)(nowMicros - lastLoopTimeMicros) / 1000000.0;
  lastLoopTimeMicros = nowMicros;

  // Cap dt
  if (pid_dt <= 0)
    pid_dt = 0.001;
  if (pid_dt > 0.05)
    pid_dt = 0.05; // Cap ved 50ms for at undgå store D-term ryk ved pauser

  // --- Læs Sensorer (fra IMU Manager) ---
  imuManager.update();                                    // <--- Lad IMUManager læse reports
  double currentPitch = imuManager.getPitch();            // <--- Hent pitch
  double currentPitchRate = imuManager.getRawPitchRate(); // <--- Hent rå pitch rate
  // double currentFilteredPitchRate = imuManager.getFilteredPitchRate(); // Den filtrerede rate er tilgængelig, men ikke brugt af PID

  // --- Læs Motor Hastigheder ---
  double actualRpmLeft = speedCtrl1.getActualRpm();
  double actualRpmRight = speedCtrl2.getActualRpm();
  double currentVelocity = (actualRpmLeft + actualRpmRight) / 2.0; // Gennemsnit RPM som hastighed

  // --- BEREGN AKKUMULERET DISTANCE (TILFØJET) ---
  // Konverter gennemsnitlig RPM til lineær hastighed (m/s)
  const double avgRps = currentVelocity / 60.0;         // RPM -> Revolutions Per Second
  const double circumference = M_PI * WHEEL_DIAMETER_M; // Omkreds i meter
  const double velocity_mps = avgRps * circumference;

  // Akkumuler den absolutte distance kørt i dette tidsinterval (pid_dt)
  // Vi bruger abs(), så både frem og tilbage bevægelse tæller positivt
  netDisplacement_m += velocity_mps * pid_dt;

  // --- Opdater State Machine ---
  RobotState previousState = currentState;
  switch (currentState)
  {
  case IDLE:
    // Overgang fra IDLE sker automatisk i setup() nu
    break;
  case BALANCING:
    // Gå til FALLEN hvis hældningen er for stor
    if (abs(currentPitch - g_init_balance) > MAX_TILT_ANGLE_SAFETY)
    {
      currentState = FALLEN;
      Serial.printf("TAG_STATE: For stor hældning (%.2f) -> Går til FALLEN state!\n", currentPitch);
    }
    break;
  case FALLEN:
    // Gå til BALANCING hvis robotten er oprejst igen
    if (abs(currentPitch - g_init_balance) < RECOVERY_ANGLE_THRESHOLD)
    {
      currentState = BALANCING;
      Serial.println("TAG_STATE: Robot oprejst igen -> Går til BALANCING state.");
    }
    break;
  case CALIBRATING_IMU:
    // Not implemented yet
    break;
  }

  // Håndter state transition actions
  if (currentState != previousState)
  {
    switch (currentState)
    {
    case BALANCING:
      balanceController.resetIntegral();
      netDisplacement_m = 0.0;
      balancing_start_time = millis();
      has_saved_this_session = false;
      stability_timer_start = 0;
      break;
    case FALLEN:
      speedCtrl1.stop();
      speedCtrl2.stop();
      balanceController.resetIntegral(); // Nulstil integral ved fald
      break;
    case IDLE:
      speedCtrl1.stop();
      speedCtrl2.stop();
      balanceController.resetIntegral(); // Nulstil integral i IDLE
      break;
    default:
      break;
    }
  }

  // --- Kontrol Logik ---
  double targetRpm1 = 0.0;
  double targetRpm2 = 0.0;
  double steeringCommand = 0.0; // Placeholder for remote control/joystick input

  // Opdater Balance Controlleren - den beregner nu de ønskede motor RPM
  balanceController.update(currentPitch, currentPitchRate, currentVelocity, // <--- Brug data fra IMUManager
                           pid_dt, steeringCommand, currentState,
                           targetRpm1, targetRpm2); // targetRpm1/2 opdateres via reference

  if (currentState == BALANCING && g_balance_calib_ki != 0)
  {
    // Juster g_init_balance gradvist i HVER loop-cyklus.
    // Dette sikrer en jævn og rolig konvergens mod det rigtige balancepunkt.
    g_init_balance += balanceController.getIntegral() * g_balance_calib_ki * pid_dt;
  }
  if (!has_saved_this_session && currentState == BALANCING)
  {

    if (millis() - balancing_start_time > MIN_BALANCE_TIME_BEFORE_SAVE_CHECK_MS)
    {
      double current_i_term = balanceController.iTerm_log;
      if (abs(current_i_term) < STABILITY_ITERM_THRESHOLD)
      {
        // Ja, det er stabilt LIGE NU. Start/fortsæt stabilitets-timeren.
        if (stability_timer_start == 0)
        {
          stability_timer_start = millis(); // Start timeren første gang vi ser stabilitet
        }

        // Tjek 3: Har det været stabilt længe nok?
        if (millis() - stability_timer_start > STABILITY_MIN_DURATION_MS)
        {
          // JA! Systemet er nu officielt stabilt. Tag beslutning om at gemme.
          has_saved_this_session = true; // "Lås" så vi ikke gemmer igen.

          if (abs(g_init_balance - nvs_g_init_balance) > SAVE_THRESHOLD)
          {
            Serial.printf("TAG_INFO: System stable. Auto-saving new g_init_balance: %.4f\n", g_init_balance);
            saveTuningParameters();
          }
          else
          {
            Serial.println("TAG_INFO: System stable. Change was small, not saving.");
          }
        }
      }
      else
      {
        // Nej, I-leddet er stort. Systemet er ustabilt. Nulstil stabilitets-timeren.
        stability_timer_start = 0;
      }
    }
    if (prt_timer_start == 0)
    {
      prt_timer_start = millis();
    }
    if (prt_timer_start > 0 && millis() - prt_timer_start > 5000)
    {
      // Hvis det har været ustabilt i mere end 5 sekunder, så nulstil I-leddet
      Serial.printf("TAG_WARNING: System unstable. %.4f\n", balanceController.iTerm_log);
      prt_timer_start = millis(); // Nulstil timeren
    }
  }

  // --- Send kommandoer til Speed Controllers ---
  speedCtrl1.setTargetRpm(targetRpm1);
  speedCtrl2.setTargetRpm(targetRpm2);

  csvLogger.logData(nowMicros,
                    currentPitch,
                    currentPitchRate,
                    balanceController.balanceCmd_log,
                    balanceController.pTerm_log,
                    balanceController.iTerm_log,
                    balanceController.dTerm_log,
                    balanceController.scaled_output_log,
                    netDisplacement_m);
}
// End loop