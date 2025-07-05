#include "BalanceController.h"
#include "config.h"  // Inkluderer nu RobotState enum
#include <Arduino.h> // Til constrain, abs etc.
#include <cmath>     // Til abs, M_PI etc.

// --- Globale Tuning Variable (Deklarationer fra tuning_handler.h/config.h) ---
extern double g_balance_kp;
extern double g_balance_ki;
extern double g_balance_kd;
extern double g_init_balance;
extern double g_balance_output_to_rpm_scale;
extern double g_power_gain;
extern double g_velocity_kp;

// --- Konstruktør ---
BalanceController::BalanceController() : _pitch_error_integral(0.0),
                                         pTerm_log(0.0),
                                         iTerm_log(0.0),
                                         dTerm_log(0.0),
                                         balanceCmd_log(0.0),
                                         scaled_output_log(0.0) // <--- Initialiser ny log variabel
{
}

// --- Initialisering ---
void BalanceController::begin()
{
  resetIntegral();         // Start med nul integral
  scaled_output_log = 0.0; // Nulstil også denne
}

// --- Nulstil Integral ---
void BalanceController::resetIntegral()
{
  _pitch_error_integral = 0.0;
  iTerm_log = 0.0; // Nulstil også log-variablen
}

// --- Hovedkontrolfunktion ---
void BalanceController::update(double currentPitch, double currentPitchRate, double currentVelocity,
                               double dt, double steeringCmd, RobotState currentState,
                               double &targetRpmLeft, double &targetRpmRight)
{
  // Nulstil outputs og log variabler hvis ikke i BALANCING state
  if (currentState != BALANCING)
  {
    targetRpmLeft = 0.0;
    targetRpmRight = 0.0;
    pTerm_log = 0.0;
    // _pitch_error_integral beholder sin værdi, men akkumuleres ikke
    // iTerm_log opdateres ikke
    dTerm_log = 0.0;
    balanceCmd_log = 0.0;
    scaled_output_log = 0.0; // <--- Nulstil log
    return;
  }

  // --- Balance PID Beregning ---
  double pitch_error = currentPitch - g_init_balance;

  // P Term
  pTerm_log = g_balance_kp * pitch_error;

  // I Term (akkumuler kun når tæt på balance, med anti-windup)
  if (abs(pitch_error) < ANTI_WINDUP_ANGLE_THRESHOLD)
  {
    _pitch_error_integral += pitch_error * dt;

    // Anti-windup constrain integralen
    if (g_balance_ki != 0)
    {
      double integral_limit = BALANCE_PID_OUTPUT_LIMIT / abs(g_balance_ki);
      _pitch_error_integral = constrain(_pitch_error_integral, -integral_limit, integral_limit);
    }
    else
    {
      _pitch_error_integral = 0;
    }
  }
  iTerm_log = g_balance_ki * _pitch_error_integral;

  // D Term: BRUG DEN RÅ FUSEREDE VINKELHASTIGHED
  dTerm_log = g_balance_kd * currentPitchRate;

  // --- Velocity Control (P-term) ---
  double targetVelocity = 0.0;
  double velocityError = targetVelocity - currentVelocity;
  double velocityCorrection = velocityError * g_velocity_kp;

  // --- Samlet RÅ Kontrol Output ---
  // double raw_control_output = pTerm_log + iTerm_log + dTerm_log + velocityCorrection;

  // Konstrain det rå kontrol output
  // raw_control_output = constrain(raw_control_output, -BALANCE_PID_OUTPUT_LIMIT, BALANCE_PID_OUTPUT_LIMIT);
  // balanceCmd_log = raw_control_output; // Gem den begrænsede rå output til log

  // --- Anvend Power Gain og Skalering ---

  //---------------------------------
  // --- NY KONTROL-STRUKTUR: Adskil Kraft og Dæmpning ---

// 1. Beregn den primære oprettende kraft fra P og I leddene.
double PI_output = pTerm_log + iTerm_log;

// 2. Anvend den ikke-lineære boost KUN på opretnings-kraften.
const double pitch_error_rad = abs(pitch_error) * M_PI / 180.0;
double boost_multiplier = 1.0 + sin(pitch_error_rad) * g_power_gain;
double boosted_PI_output = PI_output * boost_multiplier;

// 3. Læg den lineære dæmpning (D-led) og hastighedskorrektion til BAGEFTER.
double raw_control_output = boosted_PI_output + dTerm_log + velocityCorrection;

// 4. Konstrain og skaler det samlede output. Bemærk: boost_multiplier er allerede anvendt.
raw_control_output = constrain(raw_control_output, -BALANCE_PID_OUTPUT_LIMIT, BALANCE_PID_OUTPUT_LIMIT);
balanceCmd_log = raw_control_output; 

double scaled_output = raw_control_output * g_balance_output_to_rpm_scale; // Fjernet boost herfra
scaled_output_log = scaled_output;
  //---------------------------------

  // --- Anvend Fysik-baseret Power Gain og Skalering ---
  // const double pitch_error_rad = abs(pitch_error) * M_PI / 180.0;
  // double boost_multiplier = 1.0 + sin(pitch_error_rad) * g_power_gain;

  // double scaled_output = raw_control_output * g_balance_output_to_rpm_scale * boost_multiplier;
  // scaled_output_log = scaled_output;

  // --- Beregn Mål RPM for Motorer (inkl. styring) ---
  targetRpmLeft = scaled_output - steeringCmd;
  targetRpmRight = scaled_output + steeringCmd;

  // Sikkerhedskonstrain på de endelige RPM kommandoer
  targetRpmLeft = constrain(targetRpmLeft, -MAX_RPM, MAX_RPM);
  targetRpmRight = constrain(targetRpmRight, -MAX_RPM, MAX_RPM);
}