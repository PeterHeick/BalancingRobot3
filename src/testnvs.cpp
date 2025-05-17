// Eksempel på NVS erase sketch (kun til at køre én gang)
#include <Arduino.h>
#include "nvs_flash.h" // For nvs_flash_erase
#include "nvs.h"       // For nvs_flash_init

void testnvs() {
  Serial.begin(115200);
  esp_err_t err = nvs_flash_erase();
  if (err == ESP_OK) {
    Serial.println("NVS partition erased successfully.");
  } else {
    Serial.printf("Error erasing NVS partition: %s\n", esp_err_to_name(err));
  }
  // Det kan være nødvendigt at geninitialisere efter sletning
  // for at andre dele af systemet kan bruge NVS igen med det samme.
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition er muligvis fuld eller versionen er forkert,
    // slet og geninitialiser.
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  Serial.println("NVS re-initialized (if needed). Restarting ESP32 now.");
  delay(1000);
  ESP.restart(); // Genstart for at sikre ren NVS-tilstand
}