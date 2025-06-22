#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Adafruit_BNO08x.h> // Inkluderer det eksterne bibliotek
#include "config.h"          // Til IMU_REPORT_INTERVAL_US og ALPHA for filter
#include "ESP32.h"           // Til I2C_SDA/SCL

class IMUManager {
private:
    Adafruit_BNO08x _bno085; // Instans af Adafruit biblioteket
    sh2_SensorValue_t _sensorValue; // Til at læse reports

    double _fusedPitch = 0.0;        // Seneste Pitch vinkel (grader)
    double _fusedPitchRate = 0.0;    // Seneste rå Pitch vinkelhastighed (grader/sek)
    double _filteredPitchRate = 0.0; // Seneste filtrerede Pitch vinkelhastighed (grader/sek)

public:
    IMUManager(); // Konstruktør

    // Initialisering - opsætter BNO085
    bool begin();

    // Opdatering - læser nye sensor reports og opdaterer interne variabler
    void update();

    // Getters til de seneste sensorværdier
    double getPitch() const { return _fusedPitch; }
    double getRawPitchRate() const { return _fusedPitchRate; }
    double getFilteredPitchRate() const { return _filteredPitchRate; }
};

#endif // IMU_MANAGER_H