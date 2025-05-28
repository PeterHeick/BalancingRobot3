/**
 * main.cpp
 * Hovedprogram for Balancerobot med Fused Sensor Data og Manuel PID kontrol.
 */

#include <Arduino.h>
#include <cmath>
#include "config.h"          // Konfigurationskonstanter (inkluderer extern deklarationer for g_)
#include "ESP32.h"           // Pin definitioner
#include "Motor.h"           // Grundlæggende Motor klasse
#include "SpeedController.h" // Hastighedsregulator klasse

// Inkluder standard Adafruit BNO08x header. Dette giver adgang til klassen
// Adafruit_BNO08x og SH2_* definitionerne.
#include <Adafruit_BNO08x.h>

// FJERN inklusionen af din egen BNO085 wrapper
// #include "BNO085.h" // <--- FJERNES

// MotorData.h og tuning_handler.h er stadig nødvendige
#include "tuning_handler.h" // Håndterer seriel tuning

// --- Globale Objekter ---
// Sørg for MOTOR_MIN_MEASUREMENT_TIME_MS er defineret i config.h
Motor motor1(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENA, MOTOR1_HALL_A, PWM_CHANNEL1, MOTOR_MIN_MEASUREMENT_TIME_MS);
Motor motor2(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_ENB, MOTOR2_HALL_A, PWM_CHANNEL2, MOTOR_MIN_MEASUREMENT_TIME_MS);

SpeedController speedCtrl1(motor1);
SpeedController speedCtrl2(motor2);

// Opret BNO085 IMU objektet DIREKTE fra Adafruit klassen
Adafruit_BNO08x bno085; // <-- RETTET TYPE

sh2_SensorValue_t sensorValue; // Til at læse BNO085 rapporter

// --- Globale PID Variable og Tilstand ---
// Disse variables DEFINITIONER er flyttet til tuning_handler.cpp.
// De er kun deklareret som 'extern' i config.h for at være globale.
// Deres værdier initialiseres i initializeTuningParameters().

// Variabler til at holde de seneste sensorværdier
double fusedPitch = 0.0;        // Fused Pitch vinkel fra BNO085 (grader)
double fusedPitchRate = 0.0;    // Fused Pitch vinkelhastighed fra BNO085 (grader/sek)
double filteredPitchRate = 0.0; // Fused Pitch vinkelhastighed fra BNO085 (grader/sek)

double pitch_error_integral = 0.0; // Manuel integral akkumulator
double balanceCmd = 0.0;           // Det samlede PID output før Power Gain/scaling

// Variabler til logning af individuelle PID termer (manuelt beregnet)
double pTerm_log = 0.0;
double iTerm_log = 0.0;
double dTerm_log = 0.0;

// g_enable_csv_output er nu defineret i tuning_handler.cpp og deklareret extern i tuning_handler.h
// bool g_enable_csv_output = false; // <-- FJERN DENNE DEFINITION HER

// --- Globale Tilstandsvariabler ---
enum RobotState
{
  IDLE,
  CALIBRATING_IMU, // Måske ikke nødvendigt med BNO085's auto-kalibrering, men god at have
  BALANCING,
  FALLEN
};
RobotState currentState = IDLE;
const char *RobotStateString[] = {"IDLE", "CALIBRATING_IMU", "BALANCING", "FALLEN"};

unsigned long lastLoopTimeMicros = 0;

// --- ISR Funktioner ---
void IRAM_ATTR motor1_isrA() { motor1.incrementPulseCount(); }
void IRAM_ATTR motor2_isrA() { motor2.incrementPulseCount(); }

// --- Hjælpefunktion til at beregne mål RPM ---
// balanceInput bruges her til Power Gain scaling - den bør være error signalet
// balanceCmd er det rå P+I+D output
void calculateTargetRpms(double pitch_error, double raw_pid_output, double steeringCmd, double &targetRpm1, double &targetRpm2)
{
  // Boost multiplier baseret på den absolutte vinkelfejl
  // Bruger pitch_error (fused pitch - InitBalance) som input til boost
  double boost_multiplier = 1.0 + abs(pitch_error) * g_power_gain;

  // Skaler RAW PID output med den normale scale OG boost_multiplier
  double scaled_pid_output = raw_pid_output * g_balance_output_to_rpm_scale * boost_multiplier;

  targetRpm1 = scaled_pid_output - steeringCmd;
  targetRpm2 = scaled_pid_output + steeringCmd;

  // Sikkerhedskonstrain på output til motorstyringen (typisk PWM eller RPM grænse)
  targetRpm1 = constrain(targetRpm1, -MAX_RPM, MAX_RPM);
  targetRpm2 = constrain(targetRpm2, -MAX_RPM, MAX_RPM);
}

// --- Setup ---
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("\n\n--- Balancerobot V4 (BNO085 Fused Data & Manual PID) ---");
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());

  initializeTuningParameters(); // Hent/initialiser tuning værdier fra NVS

  // Initialiser hardware
  Serial.println("Initialiserer motorer...");
  motor1.begin();
  motor2.begin();
  Serial.println("Motorer OK.");

  Serial.println("Initialiserer BNO085 IMU...");
  // Kald den korrekte begin metode fra Adafruit klassen (f.eks. begin_I2C)
  // Standard I2C adresse er 0x4A, men den kan være anderledes på nogle breakouts (0x4B)
  // Tjek dit breakout board's dokumentation. 0x4A er default i Adafruit bib.
  if (!bno085.begin_I2C()) // <-- RETTET KALDET
  {
    Serial.println("BNO085 ikke fundet eller initialisering fejlede via I2C. Check wiring og adresse (default 0x4A)! Prøv evt 0x4B.");
    Serial.println("Sørg for at have installeret Adafruit_BNO08x biblioteket.");
    while (1)
      delay(100); // Hold programmet hvis IMU ikke virker
  }
  Serial.println("BNO085 OK.");

  // Aktiver BNO085 rapporter
  // SH2_GAME_ROTATION_VECTOR er god til spil/robotter da den ignorerer magnetiske forstyrrelser
  // Sørg for IMU_REPORT_INTERVAL_US er defineret i config.h
  // Sørg for SH2_GAME_ROTATION_VECTOR er defineret (via Adafruit_BNO08x.h)
  if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US))
  {
    Serial.println("Fejl ved aktivering af Game Rotation Vector rapport!");
    // Fortsæt, men uden vinkeldata - ikke godt for balance!
  }
  // Aktiver Calibrated Gyro rapport for D-term
  if (!bno085.enableReport(SH2_CAL_GYRO, IMU_REPORT_INTERVAL_US))
  {
    Serial.println("Fejl ved aktivering af Calibrated Gyro rapport!");
    // Fortsæt, men D-termen baseret på rate vil ikke virke
  }
  delay(100); // Giv sensoren tid til at starte rapporter

  Serial.println("Initialiserer Speed Controllers...");
  speedCtrl1.begin();
  speedCtrl2.begin();
  Serial.println("Speed Controllers OK.");

  Serial.println("Manuel Balance PID klar.");

  Serial.println("Tilknytter Interrupts...");
  attachInterrupt(digitalPinToInterrupt(MOTOR1_HALL_A), motor1_isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_HALL_A), motor2_isrA, RISING);
  Serial.println("Interrupts OK.");

  Serial.println("\nSetup færdig. Type 'print' for current tunings or 'kp=value' etc. to set.");
  Serial.println("Use 'save' to store current tunings to NVS. Use 'init_now' to set initial balance angle.");
  lastLoopTimeMicros = micros();
  currentState = IDLE; // Start i IDLE indtil klar

  // Vent kort og start BALANCING state for at give tid til seriel monitor etc.
  delay(2000);
  // Sæt CSV header - nu med fused rate
  // Tjek om CSV output er enabled fra start (via NVS/default)
  if (g_enable_csv_output)
  {
    Serial.println("time_ms,fusedPitch,fusedPitchRate,balanceCmd,pTerm,iTerm,dTerm,scaledOutput");
  }

  // Sæt den initiale balance vinkel baseret på nuværende pitch FØR vi går i BALANCING state
  // Læs sensoren en gang for at få en startværdi
  // Prøv at læse indtil du får en Rotation Vector inden for en timeout
  unsigned long start_time = millis();
  bool got_pitch = false;
  double startupPitch = g_init_balance; // Start med den værdi fra NVS/default
  while (millis() - start_time < 2000 && !got_pitch)
  { // Vent max 2 sekunder på første RV
    while (bno085.getSensorEvent(&sensorValue))
    {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR || sensorValue.sensorId == SH2_ROTATION_VECTOR)
      {
        // Beregn Pitch vinkel fra Quaternion
        double qw = sensorValue.un.gameRotationVector.real;
        double qx = sensorValue.un.gameRotationVector.i;
        double qy = sensorValue.un.gameRotationVector.j;
        double qz = sensorValue.un.gameRotationVector.k;
        double t2 = +2.0 * (qw * qy - qz * qx); // Pitch term
        t2 = constrain(t2, -1.0, 1.0);          // Sikkerhed
        double pitchRad = asin(t2);
        startupPitch = pitchRad * 180.0 / M_PI; // Gem i grader
        got_pitch = true;
        break; // Stop reading reports for now
      }
    }
    if (!got_pitch)
      delay(50); // Vent kort hvis ingen rapport endnu
  }

  if (got_pitch)
  {
    g_init_balance = startupPitch; // Sæt init balance til den fundne pitch
    Serial.printf("TAG_INFO: Initial balance angle set to current pitch after setup: %.4f\n", g_init_balance);
  }
  else
  {
    Serial.println("ADVARSEL: Kunne ikke få start pitch fra BNO085 indenfor timeout. Bruger NVS/default g_init_balance.");
  }
  printCurrentTunings(); // Print de endelige tuning værdier

  currentState = BALANCING; // Nu klar til at starte balancing
}

// --- Loop ---
void loop()
{
  // handleSerialTuning modtager nu aktuel pitch som parameter
  handleSerialTuning(fusedPitch); // <-- RETTET KALDET MED PARAMETER

  unsigned long nowMicros = micros();
  // Beregn tid siden sidste PID/kontrol cyklus
  double pid_dt = (double)(nowMicros - lastLoopTimeMicros) / 1000000.0;
  lastLoopTimeMicros = nowMicros;

  // Undgå division med nul eller store udsving hvis loop pauser.
  // Cap ved en maximal dt (f.eks. 50ms).
  if (pid_dt <= 0 || pid_dt > 0.05)
  {
    if (pid_dt <= 0)
      pid_dt = 0.001; // Minimum 1ms hvis 0 (f.eks. første loop)
    if (pid_dt > 0.05)
      pid_dt = 0.05; // Cap ved 50ms
                     // Serial.printf("ADVARSEL: Loop dt usædvanlig: %.4f s\n", pid_dt); // Debug
  }

  // --- Læs Sensorer (BNO085) ---
  // Læs ALLE tilgængelige rapporter i denne cyklus.
  while (bno085.getSensorEvent(&sensorValue))
  {
    switch (sensorValue.sensorId)
    {
    case SH2_GAME_ROTATION_VECTOR:
    case SH2_ROTATION_VECTOR:
    {
      // Beregn Pitch vinkel fra Quaternion
      double qw = sensorValue.un.gameRotationVector.real;
      double qx = sensorValue.un.gameRotationVector.i;
      double qy = sensorValue.un.gameRotationVector.j;
      double qz = sensorValue.un.gameRotationVector.k;
      // Formel for Pitch (rotation omkring Y) fra Quaternion
      double t2 = +2.0 * (qw * qy - qz * qx); // Pitch term
      t2 = constrain(t2, -1.0, 1.0);          // Sikkerhed
      double pitchRad = asin(t2);
      fusedPitch = pitchRad * 180.0 / M_PI; // Gem i grader
      // Serial.printf("RV: %.2f\n", fusedPitch); // Debug print
      break;
    }
    case SH2_CAL_GYRO:
    {
      // Læs kalibreret vinkelhastighed (Gyro data)
      // Pitch rate er typisk rotation omkring Y-aksen (rad/s)
      fusedPitchRate = sensorValue.un.gyroscope.y * 180.0 / M_PI; // Rad/s
      LOWPASSFILTER(fusedPitchRate, filteredPitchRate, ALPHA);
      // Serial.printf("GyroY: %.2f\n", fusedPitchRate); // Debug print
      break;
    }
      // Tilføj andre cases hvis du læser andre rapporter (f.eks. SH2_LINEAR_ACCELERATION for hastighed)
    }
  }

  // --- Opdater State Machine ---
  // Bruger den seneste fusedPitch for state check
  // Kun tjek hvis vi er i BALANCING for at undgå at trigge FALLEN fra IDLE/CALIBRATING
  if (currentState == BALANCING)
  {
    if (abs(fusedPitch - g_init_balance) > MAX_TILT_ANGLE_SAFETY) // Sørg for MAX_TILT_ANGLE_SAFETY er defineret i config.h
    {
      currentState = FALLEN;
      Serial.printf("TAG_FALLEN: FEJL: For stor hældning (%.2f) -> Går til FALLEN state!\n", fusedPitch);
      // Stop motorerne med det samme når den falder
      speedCtrl1.stop();
      speedCtrl2.stop();
      // Nulstil integralen ved fald
      pitch_error_integral = 0.0;
      balanceCmd = 0.0;
    }
  }

  // Tjek om robotten er oprejst igen efter et fald
  if (currentState == FALLEN)
  {
    if (abs(fusedPitch - g_init_balance) < RECOVERY_ANGLE_THRESHOLD) // Sørg for RECOVERY_ANGLE_THRESHOLD er defineret i config.h
    {
      Serial.println("TAG_INFO: Robot oprejst igen -> Går til BALANCING state.");
      currentState = BALANCING;
      // Nulstil integralen ved genoprettelse for at undgå start-ryk
      pitch_error_integral = 0.0;
      balanceCmd = 0.0; // Nulstil kommando
    }
  }
  // Tilføj evt. IDLE->BALANCING overgangslogik hvis du har en knap/kommando til at starte

  // --- Kontrol Logik ---
  double targetRpm1 = 0.0;
  double targetRpm2 = 0.0;
  double steeringCommand = 0.0; // Hent fra input (joystick/remote) hvis relevant. For nu 0.0.

  switch (currentState)
  {
  case BALANCING:
  { // Brug scope for at definere variabler her
    // Beregn fejlen
    double pitch_error = fusedPitch - g_init_balance;
    double actualRpmLeft = speedCtrl1.getActualRpm();
    double actualRpmRight = speedCtrl2.getActualRpm();
    double horizontalVelocity = (actualRpmLeft + actualRpmRight) / 2.0; // Gennemsnit RPM

    // --- Manuel PID Beregning ---
    // P Term
    pTerm_log = g_balance_kp * pitch_error;

    // I Term (akkumuler kun når tæt på balance, med anti-windup)
    if (abs(pitch_error) < ANTI_WINDUP_ANGLE_THRESHOLD) // Sørg for ANTI_WINDUP_ANGLE_THRESHOLD er defineret i config.h
    {
      pitch_error_integral += pitch_error * pid_dt;

      // Anti-windup constrain integralen
      if (g_balance_ki != 0)
      {
        double integral_limit = BALANCE_PID_OUTPUT_LIMIT / abs(g_balance_ki); // Sørg for BALANCE_PID_OUTPUT_LIMIT er defineret i config.h
        pitch_error_integral = constrain(pitch_error_integral, -integral_limit, integral_limit);
      }
      else
      {
        pitch_error_integral = 0; // Hvis KI=0, integralen skal være 0
      }
    }
    // Hvis pitch_error er >= ANTI_WINDUP_ANGLE_THRESHOLD, akkumuleres integralen ikke.
    // Den beholder sin sidste værdi, hvilket er en form for anti-windup.

    iTerm_log = g_balance_ki * pitch_error_integral;

    // D Term: BRUG DEN FUSEREDE VINKELHASTIGHED DIREKTE FRA BNO085 (Grader/sekund)
    dTerm_log = g_balance_kd * fusedPitchRate;

    // Samlet RÅ PID output (før scaling og power gain)
    balanceCmd = pTerm_log + iTerm_log + dTerm_log;

    // Konstrain det rå PID output
    balanceCmd = constrain(balanceCmd, -BALANCE_PID_OUTPUT_LIMIT, BALANCE_PID_OUTPUT_LIMIT);

    // --- Anvend Power Gain og Beregn Motor RPM ---
    // calculateTargetRpms bruger pitch_error (fused pitch - InitBalance) til boost_multiplier
    // calculateTargetRpms bruger balanceCmd (det rå PID output efter constrain) til scaling
    calculateTargetRpms(pitch_error, balanceCmd, steeringCommand, targetRpm1, targetRpm2);

    // --- Send kommandoer til Speed Controllers ---
    // setTargetRpm håndterer selv konvertering fra RPM til PWM og sender kommandoen.
    speedCtrl1.setTargetRpm(targetRpm1);
    speedCtrl2.setTargetRpm(targetRpm2);

    break; // Afslut BALANCING case
  }

  case IDLE:
  case FALLEN:
  default:
    // Stop motorer i non-balancing states
    speedCtrl1.stop();
    speedCtrl2.stop();
    // Nulstil log-variabler for overskuelighed i plot
    pTerm_log = 0;
    iTerm_log = 0;
    dTerm_log = 0;
    balanceCmd = 0; // Nulstil også samlet kommando
    // integralen forbliver som den var, men akkumuleres ikke i IDLE/FALLEN
    break;
  }

  // --- Data Logging (til CSV) ---
  if (g_enable_csv_output)
  {
    // CSV format: tid_ms,fusedPitch,fusedPitchRate,balanceCmd,pTerm,iTerm,dTerm,scaledOutput
    Serial.printf("TAG_CSV: %.4f,", nowMicros / 1000.0); // Tid i ms
    Serial.printf("%.4f,", fusedPitch);                  // Fused Pitch (Grader)
    Serial.printf("%.4f,", fusedPitchRate);              // Fused Pitch Rate (Grader/sek)
    Serial.printf("%.4f,", balanceCmd);                  // Rå PID Output (EFTER constrain)
    Serial.printf("%.4f,", pTerm_log);                   // P-term bidrag
    Serial.printf("%.4f,", iTerm_log);                   // I-term bidrag
    Serial.printf("%.4f,", dTerm_log);                   // D-term bidrag
    // Log den faktiske scaled output sendt til SpeedControllers' setTargetRpm
    double pitch_error_for_log = fusedPitch - g_init_balance; // Genberegn pitch_error til log
    double boost_multiplier_for_log = 1.0 + abs(pitch_error_for_log) * g_power_gain;
    double scaledOutputToSend_for_log = balanceCmd * g_balance_output_to_rpm_scale * boost_multiplier_for_log;
    Serial.printf("%.4f\n", scaledOutputToSend_for_log);
  }

} // End loop()