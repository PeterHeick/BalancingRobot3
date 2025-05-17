/**
 * main.cpp
 * Hovedprogram for Balancerobot med FeedForward + PID kontrol.
 */

#include <Arduino.h>
#include <PID_v1.h>
#include <cmath>
#include "config.h"          // Konfigurationskonstanter (indeholder nu extern var)
#include "ESP32.h"           // Pin definitioner
#include "Motor.h"           // Grundlæggende Motor klasse
#include "SpeedController.h" // Hastighedsregulator klasse
#include "BNO085.h"          // IMU klasse/bibliotek
#include "MotorData.h"       // RPM->PWM lookup tabeler
#include "tuning_handler.h"  // <--- NYT INCLUDE

// --- Globale Objekter ---
Motor motor1(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENA, MOTOR1_HALL_A, PWM_CHANNEL1, MOTOR_MIN_MEASURE_TIME_MS);
Motor motor2(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_ENB, MOTOR2_HALL_A, PWM_CHANNEL2, MOTOR_MIN_MEASURE_TIME_MS);

// Opret SpeedController objekter med reference til motor og hardkodet tabel
SpeedController speedCtrl1(motor1, RPM_TO_PWM_MOTOR_1, SPEED_KP, SPEED_KI, SPEED_KD);
SpeedController speedCtrl2(motor2, RPM_TO_PWM_MOTOR_2, SPEED_KP, SPEED_KI, SPEED_KD);

BNO085 bno085;
sh2_SensorValue_t sensorValue;

double pTerm = 0.0, iTerm = 0.0, dTerm = 0.0;
double balanceSetpoint = 0.0;
double balanceInput = 0.0;
double balanceOutput = 0.0;

bool g_enable_csv_output = false;

// Opret Balance PID med de GLOBALE variable
// VIGTIGT: Disse variable (g_balance_kp etc.) skal være defineret FØR dette punkt.
// Dette sker fordi tuning_handler.cpp (hvor de defineres) linkes ind.
PID balancePID(&balanceInput, &balanceOutput, &balanceSetpoint,
               g_balance_kp, g_balance_ki, g_balance_kd, REVERSE);

// --- Globale Tilstandsvariabler ---
enum RobotState
{
  IDLE,
  CALIBRATING_IMU,
  BALANCING,
  FALLEN
};
RobotState currentState = IDLE;
const char *RobotStateString[] = {"IDLE", "CALIBRATING_IMU", "BALANCING", "FALLEN"};

unsigned long lastLoopTimeMicros = 0;
double previousFilteredBalance = 0.0;

// --- ISR Funktioner ---
void IRAM_ATTR motor1_isrA() { motor1.incrementPulseCount(); }
void IRAM_ATTR motor2_isrA() { motor2.incrementPulseCount(); }

void testnvs();

// --- Hjælpefunktion til at beregne mål RPM ---
void calculateTargetRpms(double balanceInput, double balanceCmd, double steeringCmd, double &targetRpm1, double &targetRpm2)
{
  double boost_multiplier = 1.0 + abs(balanceInput) * g_power_gain; // g_power_gain fra config/global

  // Skaler PID output (pidOutput) med den normale scale OG boost_multiplier
  double scaled_pid_output = balanceCmd * g_balance_output_to_rpm_scale * boost_multiplier;

  targetRpm1 = scaled_pid_output - steeringCmd; // Antager steeringCmd er i samme "skala" eller 0
  targetRpm2 = scaled_pid_output + steeringCmd;

  targetRpm1 = constrain(targetRpm1, -MAX_RPM, MAX_RPM);
  targetRpm2 = constrain(targetRpm2, -MAX_RPM, MAX_RPM);
}

// --- Setup ---
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("\n\n--- Balancerobot V3 (Dynamic Tuning) ---");
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());

  // testnvs();
  initializeTuningParameters(); // <--- NYT: Hent/initialiser tuning værdier

  // Initialiser hardware
  Serial.println("Initialiserer motorer...");
  motor1.begin();
  motor2.begin();
  Serial.println("Motorer OK.");

  Serial.println("Initialiserer BNO085 IMU...");
  if (!bno085.begin())
  { /* ... fejlhåndtering ... */
    while (1)
      delay(100);
  }
  if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR))
  { /* ... fejlhåndtering ... */
    while (1)
      delay(100);
  }
  Serial.println("BNO085 IMU OK.");
  delay(500);

  Serial.println("Initialiserer Speed Controllers...");
  speedCtrl1.begin();
  speedCtrl2.begin();
  Serial.println("Speed Controllers OK.");

  Serial.println("Initialiserer Balance PID...");
  // Anvend de (potentielt) indlæste tunings på PID objektet
  // Selvom PID'en blev oprettet med de globale værdier, er det godt at sikre,
  // at SetTunings kaldes efter initializeTuningParameters, hvis værdierne er ændret.
  balancePID.SetTunings(g_balance_kp, g_balance_ki, g_balance_kd);
  balancePID.SetSampleTime(LOOP_TIME_MS);
  balancePID.SetOutputLimits(-BALANCE_PID_OUTPUT_LIMIT, BALANCE_PID_OUTPUT_LIMIT);
  balancePID.SetMode(AUTOMATIC);
  Serial.println("Balance PID OK.");

  attachInterrupt(digitalPinToInterrupt(MOTOR1_HALL_A), motor1_isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_HALL_A), motor2_isrA, RISING);
  Serial.println("Interrupts OK.");

  Serial.println("\nSetup færdig. Type 'print' for current tunings or 'kp=value' etc. to set.");
  Serial.println("Use 'save' to store current tunings to NVS.");
  lastLoopTimeMicros = micros();
  currentState = IDLE;
  delay(2000);
  Serial.println("motor1,motor2,balanceInput,balanceOutput,pTerm,iTerm,dTerm");
  currentState = BALANCING;
}

// --- Loop ---
// int printInterval = 100; // Fjernet, da CSV print er kontinuerligt
void loop()
{
  handleSerialTuning(balancePID); // <--- NYT: Tjek for tuning kommandoer

  unsigned long nowMicros = micros();
  unsigned long timeElapsed = nowMicros - lastLoopTimeMicros;
  int fortegn;

  if (timeElapsed >= (LOOP_TIME_MS * 1000UL))
  {
    lastLoopTimeMicros = nowMicros;

    // --- Læs Sensorer ---
    bool imuReadSuccess = false;
    if (bno085.getSensorEvent(&sensorValue))
    {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR || sensorValue.sensorId == SH2_ROTATION_VECTOR)
      {
        double qw = sensorValue.un.gameRotationVector.real;
        double qx = sensorValue.un.gameRotationVector.i;
        double qy = sensorValue.un.gameRotationVector.j;
        double qz = sensorValue.un.gameRotationVector.k;
        double sinp = -2.0 * (qx * qz - qw * qy);
        if (sinp < -1.0)
          sinp = -1.0;
        if (sinp > 1.0)
          sinp = 1.0;
        double pitch = asin(sinp);
        double rawAngle = pitch * 180.0 / M_PI;

        // Bruger nu den globale g_init_balance
        balanceInput = LOWPASSFILTER(rawAngle + g_init_balance, previousFilteredBalance, BALANCE_FILTER_ALPHA);
        previousFilteredBalance = balanceInput;
        imuReadSuccess = true;
      }
    }

    static int imuReadErrors = 0;
    if (!imuReadSuccess)
    {
      imuReadErrors++;
      if (imuReadErrors > 10)
      {
        currentState = FALLEN;
        Serial.println("IMU Læsefejl! (For mange i træk)");
      }
      else
      {
        balanceInput = previousFilteredBalance;
        // Serial.printf("Advarsel: IMU fejl (%d/10), bruger sidste værdi.\n", imuReadErrors); // Kan være for støjende
      }
    }
    else
    {
      imuReadErrors = 0;
    }

    double steeringCommand = 0.0;

    // --- Opdater State Machine ---
    if (abs(balanceInput) > MAX_TILT_ANGLE_SAFETY && currentState == BALANCING)
    {
      currentState = FALLEN;
      Serial.printf("FEJL: For stor hældning -> Går til FALLEN state! ");
      if (balanceInput > 0)
        Serial.printf("(+ forover)\n");
      else
        Serial.printf("(- bagover)\n");
    }
    if (currentState == FALLEN)
    {
      if (abs(balanceInput) < 3.0)
      { // Juster denne grænse efter behov
        Serial.println("Robot oprejst igen -> Går til BALANCING state.");
        currentState = BALANCING;
        speedCtrl1.stop();
        speedCtrl2.stop();
        balanceOutput = 0.0;
        balancePID.SetMode(MANUAL);
        balancePID.SetTunings(g_balance_kp, g_balance_ki, g_balance_kd); // Genanvend tunings
        balancePID.SetMode(AUTOMATIC);
      }
    }

    // --- Kontrol Logik ---
    double targetRpm1 = 0.0;
    double targetRpm2 = 0.0;

    switch (currentState)
    {
    case BALANCING:
      balanceSetpoint = 0.0;
      // SetTunings kaldes nu fra handleSerialTuning hvis værdier ændres.
      // balancePID.SetTunings(g_balance_kp, g_balance_ki, g_balance_kd); // Fjern herfra, gøres kun ved ændring
      balancePID.Compute();

      pTerm = balancePID.GetPTerm();
      iTerm = balancePID.GetITerm();
      dTerm = balancePID.GetDTerm();

      calculateTargetRpms(balanceInput, balanceOutput, steeringCommand, targetRpm1, targetRpm2);

      speedCtrl1.setTargetRpm(targetRpm1);
      speedCtrl2.setTargetRpm(targetRpm2);

      fortegn = (targetRpm1 >= 0) ? 1 : -1; // Bestem retning baseret på targetRpm1
      speedCtrl1.update(fortegn);
      speedCtrl2.update(fortegn);
      break;

    case IDLE:
    case FALLEN:
    default:
      speedCtrl1.stop();
      speedCtrl2.stop();
      pTerm = 0;
      iTerm = 0;
      dTerm = 0; // Nulstil for plot
      break;
    }

    if (g_enable_csv_output)
    {
      // Kontinuerlig CSV output
      Serial.print(balanceInput, 4);
      Serial.print(",");
      Serial.print(balanceOutput, 4);
      Serial.print(",");
      Serial.print(pTerm, 4);
      Serial.print(",");
      Serial.print(iTerm, 4);
      Serial.print(",");
      Serial.print(dTerm, 4);
      Serial.println();
      Serial.printf("\n");
    }

  } // End timed loop
} // End loop()