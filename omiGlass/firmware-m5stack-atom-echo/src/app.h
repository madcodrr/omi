#ifndef APP_H
#define APP_H

#include <Arduino.h>
#include "config.h"

// Function declarations
void setup_app();
void loop_app();

// Audio functions
void setupAudio();
void startAudioCapture();
void stopAudioCapture();
void processAudioData();

// BLE functions
void setupBLE();
void updateBLEStatus();

// Button and LED functions
void setupButton();
void setupLED();
void handleButton();
void updateLED();
void setLEDColor(uint32_t color);

// Power management
void enterPowerSave();
void exitPowerSave();
void checkBattery();

#endif // APP_H
