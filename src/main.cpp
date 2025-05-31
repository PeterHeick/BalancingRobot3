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

// Inkluder standard Adafruit BNO08x header.
#include <Adafruit_BNO08x.h>

#include "tuning_handler.h" // Håndterer seriel tuning
#include "PositionController.h" // Position control klasse

// --- Globale Objekter ---
// Sørg for MOTOR_MIN_MEASUREMENT_TIME_MS er defineret i config.h
Motor motor1(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENA, MOTOR1_HALL_A, PWM_CHANNEL1, MOTOR_MIN_MEASUREMENT_TIME_MS);
Motor motor2(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_ENB, MOTOR2_HALL_A, PWM_CHANNEL2, MOTOR_MIN_MEASUREMENT_TIME_MS);

SpeedController speedCtrl1(motor1);
SpeedController speedCtrl2(motor2);

// Position controller
PositionController positionCtrl(&motor1, &motor2);

// Opret BNO085 IMU objektet DIREKTE fra Adafruit klassen
Adafruit_BNO08x bno085;

sh2_SensorValue_t sensorValue; // Til at læse BNO085 rapporter

// --- Globale PID Variable og Tilstand ---
// Disse variables DEFINITIONER er flyttet til tuning_handler.cpp.
// De er kun deklareret som 'extern' i config.h for at være globale.
// Deres værdier initialiseres i initializeTuningParameters().

// ---- Globale Tuning Variable (Definitioner) ----
// FJERN DEFINITIONERNE HER - De er defineret i tuning_handler.cpp
// double g_balance_kp = 12.0; // FJERN
// double g_balance_ki = 1.0;   // FJERN
// double g_balance_kd = 0.54;  // FJERN
// double g_velocity_kp = 0.01; // FJERN - Fjernet også fra brug og tuning_handler
// double g_init_balance = 0.8000; // FJERN
// double g_balance_output_to_rpm_scale = 1.0; // FJERN
// double g_power_gain = 0.0;   // FJERN
// Position PID variable
// double g_position_kp = 1.0; // FJERN
// double g_position_ki = 0.0; // FJERN
// double g_position_kd = 0.1; // FJERN
// double g_position_output_to_pitch_scale = 1.0; // FJERN

double pitch_error_integral = 0.0; // Manuel integral akkumulator
double balanceCmd = 0.0;           // Det samlede RÅ Balance PID output (efter constrain)

// Variabler til logning af individuelle Balance PID termer (manuelt beregnet)
double pTerm_log = 0.0;
double iTerm_log = 0.0;
double dTerm_log = 0.0;

// Variabler til logning af Position PID data
double currentPosition_log = 0.0;
double positionSetpoint_log = 0.0;
double positionError_log = 0.0;
double positionOutput_log = 0.0; // Rå/clamped output fra Position PID
double positionCorrection_log = 0.0; // Pitch offset baseret på positionOutput

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
RobotState currentState = IDLE; // Definér currentState her
const char *RobotStateString[] = {"IDLE", "CALIBRATING_IMU", "BALANCING", "FALLEN"};

unsigned long lastLoopTimeMicros = 0;

// --- ISR Funktioner ---
// Disse kaldes fra hardware interrupts. Hold dem ekstremt korte.
// De kalder Motor::incrementPulseCount(), som nu skal akkumulere TOTAL pulser.
void IRAM_ATTR motor1_isrA() { motor1.incrementPulseCount(); }
void IRAM_ATTR motor2_isrA() { motor2.incrementPulseCount(); }

// --- Hjælpefunktion til at beregne mål RPM ---
// original_pitch_error bruges til Power Gain scaling (error ift. fast init_balance)
// raw_balance_output bruges til den primære RPM kommando (raw output fra Balance PID)
void calculateTargetRpms(double original_pitch_error, double constrained_raw_balance_output, double steeringCmd, double &targetRpm1, double &targetRpm2)
{
  // Boost multiplier baseret på den absolutte VINKELFEJL ift. fast init_balance
  double boost_multiplier = 1.0 + abs(original_pitch_error) * g_power_gain;

  // Skaler RÅ Balance PID output med den normale scale OG boost_multiplier
  double scaled_pid_output = constrained_raw_balance_output * g_balance_output_to_rpm_scale * boost_multiplier;

  targetRpm1 = scaled_pid_output - steeringCmd;
  targetRpm2 = scaled_pid_output + steeringCmd;

  // Sikkerhedskonstrain på output til motorstyringen (typisk RPM grænse)
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
  // Kald den korrekte begin metode fra Adafruit klassen (begin_I2C)
  if (!bno085.begin_I2C())
  {
    Serial.println("BNO085 ikke fundet eller initialisering fejlede via I2C. Check wiring og adresse (default 0x4A)! Prøv evt 0x4B.");
    Serial.println("Sørg for at have installeret Adafruit_BNO08x biblioteket.");
    while (1)
      delay(100); // Hold programmet hvis IMU ikke virker
  }
  Serial.println("BNO085 OK.");

  // Aktiver BNO085 rapporter (Rotation Vector og Gyro)
  if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US))
  {
    Serial.println("Fejl ved aktivering af Game Rotation Vector rapport!");
  }
  if (!bno085.enableReport(SH2_CAL_GYRO, IMU_REPORT_INTERVAL_US))
  {
    Serial.println("Fejl ved aktivering af Calibrated Gyro rapport!");
  }
  delay(100); // Giv sensoren tid til at starte rapporter

  Serial.println("Initialiserer Speed Controllers...");
  speedCtrl1.begin();
  speedCtrl2.begin();
  Serial.println("Speed Controllers OK.");

  Serial.println("Initialiserer Position Controller...");
  positionCtrl.begin(); // Dette kalder resetPosition()
  Serial.println("Position Controller OK.");

  Serial.println("Manuel Balance PID klar.");

  Serial.println("Tilknytter Interrupts...");
  // Antager IRAM_ATTR på ISR funktionerne, som de er.
  attachInterrupt(digitalPinToInterrupt(MOTOR1_HALL_A), motor1_isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_HALL_A), motor2_isrA, RISING);
  Serial.println("Interrupts OK.");

  Serial.println("\nSetup færdig. Type 'print' for current tunings or 'kp=value' etc. to set.");
  Serial.println("Use 'save' to store current tunings to NVS. Use 'init_now' to set initial balance angle.");
lastLoopTimeMicros = micros();
currentState = IDLE; // Start i IDLE
  // Vent kort og start BALANCING state automatisk, som ønsket.
  delay(2000);

  // Sæt den initiale balance vinkel baseret på nuværende pitch FØR vi går i BALANCING state
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
        double qw = sensorValue.un.gameRotationVector.real;
        double qx = sensorValue.un.gameRotationVector.i;
        double qy = sensorValue.un.gameRotationVector.j;
        double qz = sensorValue.un.gameRotationVector.k;
        double t2 = +2.0 * (qw * qy - qz * qx); // Pitch term
        t2 = constrain(t2, -1.0, 1.0);
        double pitchRad = asin(t2);
        startupPitch = pitchRad * 180.0 / M_PI; // Gem i grader
        got_pitch = true;
        break; // Stop reading reports for now
      }
    }
    if (!got_pitch)
      delay(10); // Vent kort hvis ingen rapport endnu
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

  // Sæt CSV header FØR vi går i BALANCING
  if (g_enable_csv_output)
  {
    // Opdateret header baseret på de loggede variabler
    Serial.println("time_ms,fusedPitch,fusedPitchRate,rawBalanceCmd,pTerm,iTerm,dTerm,scaledMotorOutput,position,positionSetpoint,positionError,positionOutput,positionPitchCorrection");
  }


  currentState = BALANCING; // Nu klar til at starte balancing automatisk
  Serial.println("TAG_INFO: Entering BALANCING state automatically.");
  // Nulstil integralen ved start for at undgå ryk
  pitch_error_integral = 0.0;
  balanceCmd = 0.0; // Nulstil kommando
  // Nulstil position setpoint og controller
  positionCtrl.resetPosition(); // Position controller nulstilles her
}

// --- Loop ---
void loop()
{
  // Kald motor RPM data opdatering FØRST hvis din Motor::updateRpmData() metode eksisterer
  // Ellers vil getActualRpm() kun opdatere RPM når den kaldes, hvilket er uregelmæssigt
  // Hvis du ikke har implementeret updateRpmData, er denne del inaktiv.
  // motor1.updateRpmData(); // KRÆVER REVISION I MOTOR KLASSEN
  // motor2.updateRpmData(); // KRÆVER REVISION I MOTOR KLASSEN

  handleSerialTuning(fusedPitch); // Håndterer seriel kommandoer, modtager aktuel pitch

  unsigned long nowMicros = micros();
  // Beregn tid siden sidste PID/kontrol cyklus
  double pid_dt = (double)(nowMicros - lastLoopTimeMicros) / 1000000.0;
  lastLoopTimeMicros = nowMicros;

  // Undgå division med nul eller store udsving hvis loop pauser.
  // Cap ved en maximal dt (f.eks. 50ms).
  if (pid_dt <= 0 || pid_dt > 0.05)
  {
    if (pid_dt <= 0) pid_dt = 0.001; // Minimum 1ms hvis 0 (f.eks. første loop)
    if (pid_dt > 0.05) pid_dt = 0.05; // Cap ved 50ms
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
      break;
    }
    case SH2_CAL_GYRO:
    {
      // Læs kalibreret vinkelhastighed (Gyro data) - Pitch rate er rotation omkring Y-aksen
      fusedPitchRate = sensorValue.un.gyroscope.y * 180.0 / M_PI; // Konverter Rad/s til Grader/s
      filteredPitchRate = LOWPASSFILTER(fusedPitchRate, filteredPitchRate, ALPHA); // Filter D-term input? Optional, men ofte godt.
      break;
    }
    // Tilføj andre cases hvis du læser andre rapporter
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
      balanceCmd = 0.0; // Nulstil kommando
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
      // Nulstil position controller
      positionCtrl.resetPosition();
    }
  }
  // Tilføj evt. IDLE->BALANCING overgangslogik via en knap/kommando hvis auto-start fjernes

  // --- Kontrol Logik ---
  double targetRpm1 = 0.0;
  double targetRpm2 = 0.0;
  double steeringCommand = 0.0; // Hent fra input (joystick/remote) hvis relevant. For nu 0.0.

  switch (currentState)
  {
  case BALANCING:
  { // Brug scope for at definere variabler her

    // Check for position reset request (sat via serial command)
    if (g_position_reset_requested) {
      positionCtrl.resetPosition();
      g_position_reset_requested = false;
    }

    // --- Position Controller Opdatering og Beregning ---
    // Position PID'ens output justerer Balance PID'ens setpoint
    positionCtrl.update(); // Opdater position og velocity
    double positionOutput = positionCtrl.computePositionPID();
    // Clamp Position PID output (dette kan også gøres inde i computePositionPID)
    positionOutput = constrain(positionOutput, -POSITION_PID_OUTPUT_LIMIT, POSITION_PID_OUTPUT_LIMIT);

    // Position PID output bruges til at generere et pitch offset til balance PID setpoint
    double positionPitchCorrection = positionOutput * g_position_output_to_pitch_scale;

    // Gem position data til CSV logging
    currentPosition_log = positionCtrl.getCurrentPosition();
    positionSetpoint_log = positionCtrl.getPositionSetpoint();
    positionError_log = positionSetpoint_log - currentPosition_log; // Log position error
    positionOutput_log = positionOutput; // Log Position PID output (constrained)
    positionCorrection_log = positionPitchCorrection; // Log den beregnede pitch korrektion

    // --- Manuel Balance PID Beregning ---
    // Balance PID fejl er forskellen mellem aktuel pitch og DET JUSTEREDE setpoint
    double adjusted_balance_setpoint = g_init_balance + positionPitchCorrection;
    double pitch_error = fusedPitch - adjusted_balance_setpoint; // FEJLEN er i forhold til det justerede setpoint!

    // P Term (Balance PID)
    pTerm_log = g_balance_kp * pitch_error;

    // I Term (Balance PID - akkumuler kun når tæt på DET JUSTEREDE balancepunkt, med anti-windup)
    // Anti-windup tærskel skal være i forhold til den AKTUELLE pitch fejl til det dynamiske setpoint
    if (abs(pitch_error) < ANTI_WINDUP_ANGLE_THRESHOLD) // Bruger fortsat ANTI_WINDUP_ANGLE_THRESHOLD
    {
      pitch_error_integral += pitch_error * pid_dt;

      // Anti-windup constrain integralen
      if (g_balance_ki != 0)
      {
        double integral_limit = BALANCE_PID_OUTPUT_LIMIT / abs(g_balance_ki);
        pitch_error_integral = constrain(pitch_error_integral, -integral_limit, integral_limit);
      }
      else
      {
        pitch_error_integral = 0; // Hvis KI=0, integralen skal være 0
      }
    }
    // Hvis pitch_error er >= ANTI_WINDUP_ANGLE_THRESHOLD, akkumuleres integralen ikke.

    iTerm_log = g_balance_ki * pitch_error_integral;

    // D Term (Balance PID): BRUG DEN FUSEREDE VINKELHASTIGHED DIREKTE FRA BNO085 (Grader/sekund)
    // D-termen bruger VINKELHASTIGHEDEN. Den er uafhængig af setpointet.
    dTerm_log = g_balance_kd * filteredPitchRate; // Brug den filtrerede rate

    // Samlet RÅ Balance PID output (før scaling og power gain)
    double raw_balance_output = pTerm_log + iTerm_log + dTerm_log;

    // Konstrain det rå Balance PID output
    // Dette output (balanceCmd) bruges til scaling/gain og logging
    balanceCmd = constrain(raw_balance_output, -BALANCE_PID_OUTPUT_LIMIT, BALANCE_PID_OUTPUT_LIMIT);

    // --- Anvend Power Gain og Beregn Motor RPM ---
    // Power Gain baseres typisk på den oprindelige pitch fejl ift. det faste g_init_balance
    double original_pitch_error_for_gain = fusedPitch - g_init_balance;
    calculateTargetRpms(original_pitch_error_for_gain, balanceCmd, steeringCommand, targetRpm1, targetRpm2);

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
    // Position logging skal også nulstilles eller holdes konstant
    currentPosition_log = 0.0; // Eller den sidste værdi? Nulstilling er ofte klarere.
    positionSetpoint_log = 0.0;
    positionError_log = 0.0;
    positionOutput_log = 0.0;
    positionCorrection_log = 0.0;
    break;
  }

  // --- Data Logging (til CSV) ---
  if (g_enable_csv_output)
  {
    // CSV format: tid_ms,fusedPitch,fusedPitchRate,rawBalanceCmd,pTerm,iTerm,dTerm,scaledMotorOutput,position,positionSetpoint,positionError,positionOutput,positionPitchCorrection
    Serial.printf("TAG_CSV: %.4f,", nowMicros / 1000.0); // Tid i ms
    Serial.printf("%.4f,", fusedPitch);                  // Fused Pitch (Grader)
    Serial.printf("%.4f,", filteredPitcohRate);              // F
    } // End loop()
