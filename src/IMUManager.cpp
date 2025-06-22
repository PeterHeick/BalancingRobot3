#include "IMUManager.h"
#include <Arduino.h> // Til constrain, M_PI, Serial
#include <cmath>     // Til M_PI

// --- Konstruktør ---
IMUManager::IMUManager() {
    // Objekter oprettes, men hardware er ikke initialiseret endnu
}

// --- Initialisering ---
bool IMUManager::begin() {
    Serial.println("Initialiserer BNO085 IMU...");
    // Kald den korrekte begin metode fra Adafruit klassen (f.eks. begin_I2C)
    // Standard I2C adresse er 0x4A, men kan være 0x4B
    // Vi bruger I2C_SDA/SCL konstanterne fra ESP32.h
    if (!_bno085.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire )) // Brug Wire som standard I2C port
    {
        Serial.println("BNO085 ikke fundet eller initialisering fejlede via I2C. Check wiring og adresse (default 0x4A)! Prøv evt 0x4B.");
        Serial.println("Sørg for at have installeret Adafruit_BNO08x biblioteket.");
        return false; // Fejl
    }
    Serial.println("BNO085 OK.");

    // Aktiver BNO085 rapporter
    // SH2_GAME_ROTATION_VECTOR er god til spil/robotter da den ignorerer magnetiske forstyrrelser
    // Sørg for IMU_REPORT_INTERVAL_US er defineret i config.h
    if (!_bno085.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US))
    {
        Serial.println("Fejl ved aktivering af Game Rotation Vector rapport!");
        // Fortsæt, men uden vinkeldata - ikke godt for balance!
    }
    // Aktiver Calibrated Gyro rapport for D-term
    if (!_bno085.enableReport(SH2_CAL_GYRO, IMU_REPORT_INTERVAL_US))
    {
        Serial.println("Fejl ved aktivering af Calibrated Gyro rapport!");
        // Fortsæt, men D-termen baseret på rate vil ikke virke
    }

    // Vent kort på første rapporter
    delay(100);

    return true; // Succes
}

// --- Opdatering - læs reports ---
void IMUManager::update() {
    // Læs ALLE tilgængelige rapporter i denne cyklus.
    while (_bno085.getSensorEvent(&_sensorValue))
    {
        switch (_sensorValue.sensorId)
        {
        case SH2_GAME_ROTATION_VECTOR:
        case SH2_ROTATION_VECTOR: // Game Rotation Vector er foretrukket, men RV er fallback/backup
        {
            // Beregn Pitch vinkel fra Quaternion
            double qw = _sensorValue.un.gameRotationVector.real;
            double qx = _sensorValue.un.gameRotationVector.i;
            double qy = _sensorValue.un.gameRotationVector.j;
            double qz = _sensorValue.un.gameRotationVector.k;
            // Formel for Pitch (rotation omkring Y) fra Quaternion
            double t2 = +2.0 * (qw * qy - qz * qx); // Pitch term
            t2 = constrain(t2, -1.0, 1.0);          // Sikkerhed
            double pitchRad = asin(t2);
            _fusedPitch = pitchRad * 180.0 / M_PI; // Gem i grader
            break;
        }
        case SH2_CAL_GYRO:
        {
            // Læs kalibreret vinkelhastighed (Gyro data)
            // Pitch rate er typisk rotation omkring Y-aksen (rad/s)
            _fusedPitchRate = _sensorValue.un.gyroscope.y * 180.0 / M_PI; // Rad/s til Grader/sek
            // Anvend lavpasfilter. ALPHA skal være defineret i config.h
            _filteredPitchRate = LOWPASSFILTER(_fusedPitchRate, _filteredPitchRate, ALPHA);
            break;
        }
        // Tilføj andre cases hvis du læser andre rapporter (f.eks. SH2_LINEAR_ACCELERATION for hastighed)
        }
    }
    // Hvis der ikke er nogen reports, beholder _fusedPitch og _fusedPitchRate deres sidste værdi.
    // Dette er normal adfærd for denne type sensorlæsning.
}

// Getters er defineret i headeren, da de er simple oneliners.