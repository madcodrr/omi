#include "app.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include "config.h"

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Device state
device_state_t deviceState = DEVICE_BOOTING;
bool deviceActive = true;

// Audio state
bool audioCapturing = false;
unsigned long lastAudioCapture = 0;
size_t audioBufferIndex = 0;
int16_t audioBuffer[AUDIO_BUFFER_SIZE];

// Button and LED state
volatile bool buttonPressed = false;
unsigned long buttonPressTime = 0;
button_state_t buttonState = BUTTON_IDLE;
led_status_t ledMode = LED_BOOT_SEQUENCE;

// Power management
unsigned long lastActivity = 0;
bool powerSaveMode = false;

// BLE state
bool connected = false;
size_t sentAudioBytes = 0;
size_t sentAudioFrames = 0;
bool audioDataUploading = false;
uint16_t packetCounter = 0;  // Global packet counter for OMI protocol

// =============================================================================
// BLE SETUP
// =============================================================================

// Device Information Service UUIDs  
#define DEVICE_INFORMATION_SERVICE_UUID (uint16_t)0x180A
#define MANUFACTURER_NAME_STRING_CHAR_UUID (uint16_t)0x2A29
#define MODEL_NUMBER_STRING_CHAR_UUID (uint16_t)0x2A24
#define FIRMWARE_REVISION_STRING_CHAR_UUID (uint16_t)0x2A26
#define HARDWARE_REVISION_STRING_CHAR_UUID (uint16_t)0x2A27

// Main OMI Service - using config.h UUIDs
static BLEUUID serviceUUID(OMI_SERVICE_UUID);
static BLEUUID audioDataUUID(AUDIO_DATA_UUID);
static BLEUUID audioCodecUUID(AUDIO_CODEC_UUID);
static BLEUUID audioControlUUID(AUDIO_CONTROL_UUID);

// BLE Characteristics
BLECharacteristic *audioDataCharacteristic;
BLECharacteristic *audioCodecCharacteristic;
BLECharacteristic *audioControlCharacteristic;
BLECharacteristic *batteryLevelCharacteristic;

// RGB LED
Adafruit_NeoPixel led(NUM_LEDS, RGB_LED_PIN, LED_TYPE);

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void handleAudioControl(int8_t controlValue);
void readBatteryLevel();
void updateBatteryService();
void IRAM_ATTR buttonISR();
void blinkLED(int count, int delayMs);
void enableLightSleep();
void sendAudioData(int16_t* data, size_t length);
void restartI2S();

// =============================================================================
// BLE CALLBACKS
// =============================================================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        connected = true;
        deviceState = DEVICE_READY;
        ledMode = LED_READY;
        Serial.println("BLE Client connected");
        lastActivity = millis();
    }

    void onDisconnect(BLEServer* pServer) {
        connected = false;
        audioCapturing = false;
        deviceState = DEVICE_READY;
        ledMode = LED_READY;
        Serial.println("BLE Client disconnected");
        
        // Restart advertising
        BLEDevice::startAdvertising();
        Serial.println("Restarted BLE advertising");
    }
};

class AudioControlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            int8_t controlValue = (int8_t)value[0];
            handleAudioControl(controlValue);
            lastActivity = millis();
        }
    }
};

// =============================================================================
// BUTTON ISR
// =============================================================================
void IRAM_ATTR buttonISR() {
    buttonPressed = true;
}

// =============================================================================
// SETUP FUNCTIONS
// =============================================================================

void setup_app() {
    Serial.begin(115200);
    Serial.println("\n=== OMI M5Stack Atom Echo Starting ===");
    
    // Initialize components
    setupLED();
    setupButton();
    setupAudio();
    setupBLE();
    
    // Setup WiFi (for potential future use)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    deviceState = DEVICE_READY;
    ledMode = LED_READY;
    lastActivity = millis();
    
    Serial.println("=== OMI M5Stack Atom Echo Ready ===");
}

void setupLED() {
    led.begin();
    led.setBrightness(LED_BRIGHTNESS);
    led.show(); // Initialize all pixels to 'off'
    Serial.println("LED initialized");
}

void setupButton() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    Serial.println("Button initialized on pin " + String(BUTTON_PIN));
}

void setupAudio() {
    Serial.println("Initializing I2S Audio...");
    
    // Enhanced I2S configuration for M5Stack Atom Echo
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = AUDIO_SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)AUDIO_BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = AUDIO_DMA_BUFFER_COUNT,
        .dma_buf_len = AUDIO_DMA_BUFFER_SIZE,
        .use_apll = true,  // Enable APLL for stable clock generation
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,  // Add MCLK configuration
        .bits_per_chan = I2S_BITS_PER_CHAN_16BIT  // Explicit bits per channel
    };
    
    // I2S pin configuration for M5Stack Atom Echo
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,     // G19 - BCLK
        .ws_io_num = I2S_LRC_PIN,       // G33 - LRCK (Word Select)
        .data_out_num = I2S_DOUT_PIN,   // G22 - DataOut SPK-I2S
        .data_in_num = I2S_DIN_PIN      // G23 - DataIn/MIC
    };
    
    // Install I2S driver with comprehensive error handling
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("❌ Failed to install I2S driver: %s\n", esp_err_to_name(err));
        ledMode = LED_ERROR;
        return;
    }
    Serial.println("✓ I2S driver installed");
    
    // Set I2S pin configuration
    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("❌ Failed to set I2S pins: %s\n", esp_err_to_name(err));
        ledMode = LED_ERROR;
        return;
    }
    Serial.println("✓ I2S pins configured");
    
    // CRITICAL: Start I2S peripheral (this was missing!)
    err = i2s_start(I2S_NUM_0);
    if (err != ESP_OK) {
        Serial.printf("❌ Failed to start I2S: %s\n", esp_err_to_name(err));
        ledMode = LED_ERROR;
        return;
    }
    Serial.println("✓ I2S peripheral started");
    
    // Clear DMA buffers to prevent initial noise
    i2s_zero_dma_buffer(I2S_NUM_0);
    Serial.println("✓ I2S DMA buffers cleared");
    
    Serial.println("🎵 I2S Audio initialization complete!");
}

void setupBLE() {
    Serial.println("Initializing BLE...");
    
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // Create OMI Service
    BLEService *pService = pServer->createService(serviceUUID);
    
    // Audio Data Characteristic
    audioDataCharacteristic = pService->createCharacteristic(
        audioDataUUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    audioDataCharacteristic->addDescriptor(new BLE2902());
    
    // Audio Codec Characteristic (read-only, returns codec type)
    audioCodecCharacteristic = pService->createCharacteristic(
        audioCodecUUID,
        BLECharacteristic::PROPERTY_READ
    );
    // Set codec type: 0=PCM16, 1=PCM8, 20=OPUS (we use PCM16)
    uint8_t codecType = 0; // PCM16
    audioCodecCharacteristic->setValue(&codecType, 1);
    
    // Audio Control Characteristic (new UUID for control commands)
    audioControlCharacteristic = pService->createCharacteristic(
        audioControlUUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );
    audioControlCharacteristic->setCallbacks(new AudioControlCallbacks());
    
    // Battery Service
    BLEService *batteryService = pServer->createService(BATTERY_SERVICE_UUID);
    batteryLevelCharacteristic = batteryService->createCharacteristic(
        BATTERY_LEVEL_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    batteryLevelCharacteristic->addDescriptor(new BLE2902());
    
    // Device Information Service
    BLEService *deviceInfoService = pServer->createService(DEVICE_INFORMATION_SERVICE_UUID);
    
    BLECharacteristic *manufacturerChar = deviceInfoService->createCharacteristic(
        MANUFACTURER_NAME_STRING_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    manufacturerChar->setValue(MANUFACTURER_NAME);
    
    BLECharacteristic *modelChar = deviceInfoService->createCharacteristic(
        MODEL_NUMBER_STRING_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    modelChar->setValue("OMI M5Stack Atom Echo");
    
    BLECharacteristic *firmwareChar = deviceInfoService->createCharacteristic(
        FIRMWARE_REVISION_STRING_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    firmwareChar->setValue(FIRMWARE_VERSION_STRING);
    
    BLECharacteristic *hardwareChar = deviceInfoService->createCharacteristic(
        HARDWARE_REVISION_STRING_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    hardwareChar->setValue(HARDWARE_REVISION);
    
    // Start services
    pService->start();
    batteryService->start();
    deviceInfoService->start();
    
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising started");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop_app() {
    // Handle button presses
    handleButton();
    
    // Update LED status
    updateLED();
    
    // Process audio if capturing
    if (audioCapturing) {
        processAudioData();
    }
    
    // Check battery level periodically
    static unsigned long lastBatteryCheck = 0;
    if (millis() - lastBatteryCheck > BATTERY_REPORT_INTERVAL_MS) {
        readBatteryLevel();
        updateBatteryService();
        lastBatteryCheck = millis();
    }
    
    // Power management
    if (millis() - lastActivity > IDLE_THRESHOLD_MS && !powerSaveMode) {
        enterPowerSave();
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}

// =============================================================================
// AUDIO FUNCTIONS
// =============================================================================

void startAudioCapture() {
    if (!audioCapturing) {
        audioCapturing = true;
        audioBufferIndex = 0;
        deviceState = DEVICE_RECORDING;
        ledMode = LED_RECORDING;
        
        // Stop I2S, clear buffers, and restart
        i2s_stop(I2S_NUM_0);
        i2s_zero_dma_buffer(I2S_NUM_0);
        esp_err_t err = i2s_start(I2S_NUM_0);
        if (err != ESP_OK) {
            Serial.printf("Failed to restart I2S for capture: %s\n", esp_err_to_name(err));
            ledMode = LED_ERROR;
            audioCapturing = false;
            return;
        }
        
        Serial.println("🎤 Audio capture started");
        lastActivity = millis();
    }
}

void stopAudioCapture() {
    if (audioCapturing) {
        // Send any remaining audio data before stopping
        if (audioBufferIndex > 0 && connected && audioDataCharacteristic) {
            sendAudioData(audioBuffer, audioBufferIndex);
            audioBufferIndex = 0;
        }
        
        audioCapturing = false;
        deviceState = DEVICE_READY;
        ledMode = LED_READY;
        
        // Stop I2S and clear buffers
        i2s_stop(I2S_NUM_0);
        i2s_zero_dma_buffer(I2S_NUM_0);
        
        Serial.println("⏹️ Audio capture stopped");
        lastActivity = millis();
    }
}

void processAudioData() {
    if (!audioCapturing) return;
    
    size_t bytesRead = 0;
    int16_t i2sBuffer[AUDIO_DMA_BUFFER_SIZE];
    static int errorCount = 0;
    
    // Read audio data from I2S with improved timeout
    esp_err_t result = i2s_read(I2S_NUM_0, i2sBuffer, sizeof(i2sBuffer), &bytesRead, pdMS_TO_TICKS(100));
    
    if (result == ESP_OK && bytesRead > 0) {
        size_t samplesRead = bytesRead / sizeof(int16_t);
        
        // Validate audio data quality
        bool hasValidAudio = false;
        int16_t maxSample = 0;
        for (size_t i = 0; i < samplesRead; i++) {
            int16_t sample = abs(i2sBuffer[i]);
            if (sample > maxSample) maxSample = sample;
            if (sample > AUDIO_VALIDATION_THRESHOLD) {
                hasValidAudio = true;
            }
        }
        
        // Only process if we have valid audio signal
        if (hasValidAudio || maxSample > 50) {
            // Copy samples to main buffer
            for (size_t i = 0; i < samplesRead && audioBufferIndex < AUDIO_BUFFER_SIZE; i++) {
                audioBuffer[audioBufferIndex++] = i2sBuffer[i];
            }
            
            errorCount = 0; // Reset error count on successful read
        }
        
        // Send data when buffer is full
        if (audioBufferIndex >= AUDIO_BUFFER_SIZE) {
            if (connected && audioDataCharacteristic) {
                sendAudioData(audioBuffer, audioBufferIndex);
            }
            audioBufferIndex = 0;
        }
        
        lastActivity = millis();
    } else {
        // Handle I2S read errors with recovery
        errorCount++;
        Serial.printf("I2S read error: %s (count: %d)\n", esp_err_to_name(result), errorCount);
        
        if (errorCount >= AUDIO_ERROR_RECOVERY_ATTEMPTS) {
            Serial.println("Too many I2S errors, attempting restart...");
            restartI2S();
            errorCount = 0;
        }
    }
}

void sendAudioData(int16_t* data, size_t length) {
    if (!connected || !audioDataCharacteristic) return;
    
    // Convert to bytes  
    uint8_t* byteData = (uint8_t*)data;
    size_t totalBytes = length * sizeof(int16_t);
    
    // Calculate max packet size (MTU minus 3-byte header)
    #define NET_BUFFER_HEADER_SIZE 3
    #define DEFAULT_MTU 517  // BLE MTU size from config
    size_t maxPacketSize = DEFAULT_MTU - NET_BUFFER_HEADER_SIZE;
    
    // Fragment and send data with proper OMI protocol headers
    size_t offset = 0;
    uint8_t chunkIndex = 0;
    uint16_t currentPacketId = packetCounter;
    packetCounter++; // Increment for next frame (wraps automatically at 65535)
    
    while (offset < totalBytes) {
        // Calculate current chunk size  
        size_t currentChunkSize = min(maxPacketSize, totalBytes - offset);
        
        // Create packet with 3-byte header + audio data
        uint8_t packet[currentChunkSize + NET_BUFFER_HEADER_SIZE];
        
        // Add OMI protocol header
        packet[0] = currentPacketId & 0xFF;        // Packet ID low byte
        packet[1] = (currentPacketId >> 8) & 0xFF; // Packet ID high byte  
        packet[2] = chunkIndex;                    // Index within current packet
        
        // Copy audio data after header
        memcpy(packet + NET_BUFFER_HEADER_SIZE, byteData + offset, currentChunkSize);
        
        // Send the packet via BLE notification
        audioDataCharacteristic->setValue(packet, currentChunkSize + NET_BUFFER_HEADER_SIZE);
        audioDataCharacteristic->notify();
        
        // Update counters
        offset += currentChunkSize;
        chunkIndex++;
        sentAudioBytes += currentChunkSize;
        
        // Small delay to prevent overwhelming the BLE stack
        if (chunkIndex > 0) delay(BLE_AUDIO_TRANSFER_DELAY);
    }
    
    sentAudioFrames++;
    
    // Reduced debug output  
    if (sentAudioFrames % 100 == 0) {
        Serial.printf("Sent %zu frames (packet %d)\n", sentAudioFrames, currentPacketId);
    }
}

// =============================================================================
// BUTTON HANDLING
// =============================================================================

void handleButton() {
    if (!buttonPressed) return;
    
    buttonPressed = false;
    unsigned long currentTime = millis();
    
    if (buttonState == BUTTON_IDLE) {
        buttonState = BUTTON_PRESSED;
        buttonPressTime = currentTime;
    }
    
    // Check for long press
    if (buttonState == BUTTON_PRESSED && 
        (currentTime - buttonPressTime) > BUTTON_LONG_PRESS_MS) {
        buttonState = BUTTON_LONG_PRESS;
        
        // Long press: toggle power save mode
        if (powerSaveMode) {
            exitPowerSave();
        } else {
            enterPowerSave();
        }
        
        Serial.println("Button long press - toggled power save");
    }
    
    // Check for button release
    if (digitalRead(BUTTON_PIN) == HIGH) {
        if (buttonState == BUTTON_PRESSED) {
            // Short press: toggle audio capture
            if (audioCapturing) {
                stopAudioCapture();
            } else {
                startAudioCapture();
            }
            Serial.println("Button short press - toggled audio capture");
        }
        
        buttonState = BUTTON_IDLE;
    }
    
    lastActivity = millis();
}

// =============================================================================
// LED FUNCTIONS
// =============================================================================

void updateLED() {
    static unsigned long lastLEDUpdate = 0;
    static bool ledState = false;
    unsigned long currentTime = millis();
    
    switch (ledMode) {
        case LED_BOOT_SEQUENCE:
            // Fast blue blink during boot
            if (currentTime - lastLEDUpdate > 200) {
                ledState = !ledState;
                setLEDColor(ledState ? LED_COLOR_BOOT : LED_COLOR_OFF);
                lastLEDUpdate = currentTime;
            }
            break;
            
        case LED_READY:
            // Solid green when ready
            setLEDColor(LED_COLOR_READY);
            break;
            
        case LED_RECORDING:
            // Solid red when recording
            setLEDColor(LED_COLOR_ACTIVE);
            break;
            
        case LED_LOW_BATTERY:
            // Slow yellow blink for low battery
            if (currentTime - lastLEDUpdate > 1000) {
                ledState = !ledState;
                setLEDColor(ledState ? LED_COLOR_LOW_BATT : LED_COLOR_OFF);
                lastLEDUpdate = currentTime;
            }
            break;
            
        case LED_ERROR:
            // Fast magenta blink for error
            if (currentTime - lastLEDUpdate > 300) {
                ledState = !ledState;
                setLEDColor(ledState ? LED_COLOR_ERROR : LED_COLOR_OFF);
                lastLEDUpdate = currentTime;
            }
            break;
            
        case LED_SLEEP_MODE:
            // Very slow dim blue blink in sleep
            if (currentTime - lastLEDUpdate > 5000) {
                ledState = !ledState;
                if (ledState) {
                    led.setBrightness(10);
                    setLEDColor(LED_COLOR_BOOT);
                } else {
                    setLEDColor(LED_COLOR_OFF);
                }
                led.setBrightness(LED_BRIGHTNESS);
                lastLEDUpdate = currentTime;
            }
            break;
            
        default:
            setLEDColor(LED_COLOR_OFF);
            break;
    }
}

void setLEDColor(uint32_t color) {
    led.setPixelColor(0, color);
    led.show();
}

// =============================================================================
// AUDIO CONTROL HANDLING
// =============================================================================

void handleAudioControl(int8_t controlValue) {
    Serial.printf("Audio control received: %d\n", controlValue);
    
    switch (controlValue) {
        case 1: // Start audio capture
            startAudioCapture();
            break;
            
        case 0: // Stop audio capture
            stopAudioCapture();
            break;
            
        case 2: // Toggle audio capture
            if (audioCapturing) {
                stopAudioCapture();
            } else {
                startAudioCapture();
            }
            break;
            
        case 10: // Enter power save
            enterPowerSave();
            break;
            
        case 11: // Exit power save
            exitPowerSave();
            break;
            
        default:
            Serial.printf("Unknown audio control value: %d\n", controlValue);
            break;
    }
}

// =============================================================================
// POWER MANAGEMENT
// =============================================================================

void enterPowerSave() {
    if (powerSaveMode) return;
    
    powerSaveMode = true;
    
    // Reduce CPU frequency
    setCpuFrequencyMhz(MIN_CPU_FREQ_MHZ);
    
    // Stop audio capture
    if (audioCapturing) {
        stopAudioCapture();
    }
    
    deviceState = DEVICE_POWER_SAVE;
    ledMode = LED_SLEEP_MODE;
    
    Serial.println("Entered power save mode");
}

void exitPowerSave() {
    if (!powerSaveMode) return;
    
    powerSaveMode = false;
    
    // Restore CPU frequency
    setCpuFrequencyMhz(NORMAL_CPU_FREQ_MHZ);
    
    deviceState = DEVICE_READY;
    ledMode = LED_READY;
    
    lastActivity = millis();
    
    Serial.println("Exited power save mode");
}

// =============================================================================
// BATTERY MANAGEMENT
// =============================================================================

void readBatteryLevel() {
    // For M5Stack Atom Echo, we'll simulate battery level based on USB power
    // In a real implementation, you would read from a battery management IC
    
    // Check if USB power is connected (simplified)
    int batteryPercentage = 100; // Assume always powered via USB
    
    // Update BLE characteristic
    if (batteryLevelCharacteristic && connected) {
        uint8_t batteryLevel = (uint8_t)batteryPercentage;
        batteryLevelCharacteristic->setValue(&batteryLevel, 1);
        batteryLevelCharacteristic->notify();
    }
    
    // Update LED if low battery (won't happen with USB power, but included for completeness)
    if (batteryPercentage < 20) {
        ledMode = LED_LOW_BATTERY;
        deviceState = DEVICE_LOW_BATTERY;
    }
}

void updateBatteryService() {
    readBatteryLevel();
}

// =============================================================================
// I2S RECOVERY FUNCTIONS
// =============================================================================

void restartI2S() {
    Serial.println("🔄 Attempting I2S restart...");
    
    // Stop current I2S operations
    i2s_stop(I2S_NUM_0);
    i2s_driver_uninstall(I2S_NUM_0);
    
    // Small delay to ensure clean shutdown
    delay(100);
    
    // Reinitialize I2S with same configuration
    setupAudio();
    
    // If we were capturing audio, restart capture
    if (audioCapturing) {
        audioBufferIndex = 0; // Reset buffer
        i2s_zero_dma_buffer(I2S_NUM_0);
        Serial.println("🔄 I2S restarted, resuming audio capture");
    }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

void blinkLED(int count, int delayMs) {
    for (int i = 0; i < count; i++) {
        setLEDColor(LED_COLOR_BOOT);
        delay(delayMs);
        setLEDColor(LED_COLOR_OFF);
        delay(delayMs);
    }
}

void enableLightSleep() {
    // Configure light sleep (this is a simplified implementation)
    if (powerSaveMode && !audioCapturing) {
        esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_DURATION_US);
        esp_light_sleep_start();
    }
}
