#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// BOARD CONFIGURATION - M5Stack Atom Echo
// =============================================================================
#define CAMERA_MODEL_M5STACK_ATOM_ECHO  // Define board model for M5Stack Atom Echo
#define BOARD_HAS_MEMORY_CARD false     // No SD card support
#define CONFIG_ARDUHAL_ESP_LOG          // Enable Arduino HAL logging

// =============================================================================
// DEVICE CONFIGURATION
// =============================================================================
#define BLE_DEVICE_NAME "Omi"
#define FIRMWARE_VERSION_STRING "1.0.0"
#define HARDWARE_REVISION "ESP32-PICO-v1.0"
#define MANUFACTURER_NAME "Based Hardware"

// =============================================================================
// M5STACK ATOM ECHO PIN DEFINITIONS
// =============================================================================
// I2S Audio Configuration (as specified in the requirements)
#define I2S_DOUT_PIN      22    // G22 - DataOut SPK-I2S
#define I2S_BCLK_PIN      19    // G19 - BCLK SPK-I2S  
#define I2S_DIN_PIN       23    // G23 - DataIn/MIC
#define I2S_LRC_PIN       33    // G33 - LRCK (Word Select)

// Control Pins
#define BUTTON_PIN        39    // G39 - Button
#define RGB_LED_PIN       27    // G27 - RGB LED (NeoPixel)

// I2S Configuration
#define I2S_SAMPLE_RATE   16000  // 16kHz sample rate for audio
#define I2S_BITS_PER_SAMPLE 16   // 16-bit samples
#define I2S_CHANNELS      1      // Mono audio
#define I2S_BUFFER_SIZE   1024   // Buffer size for I2S

// =============================================================================
// POWER MANAGEMENT - Optimized for M5Stack Atom Echo
// =============================================================================
// CPU Frequency Management
#define MAX_CPU_FREQ_MHZ 240      // ESP32-PICO can run at full speed
#define MIN_CPU_FREQ_MHZ 80       // Minimum for stable I2S operation
#define NORMAL_CPU_FREQ_MHZ 160   // Normal operation frequency

// Sleep Management
#define LIGHT_SLEEP_DURATION_US 50000   // 50ms light sleep intervals
#define DEEP_SLEEP_THRESHOLD_MS 300000  // 5 minutes of inactivity triggers deep sleep
#define IDLE_THRESHOLD_MS 45000         // 45 seconds to enter power save mode

// Battery Configuration (if external battery is connected)
#define BATTERY_MAX_VOLTAGE 5.0f        // 5V USB power
#define BATTERY_MIN_VOLTAGE 4.5f        // Minimum stable voltage
#define BATTERY_REPORT_INTERVAL_MS 90000 // 1.5 minute reporting

// =============================================================================
// AUDIO CONFIGURATION - I2S Audio Processing
// =============================================================================
#define AUDIO_SAMPLE_RATE 16000         // 16kHz for voice processing
#define AUDIO_BITS_PER_SAMPLE 16        // 16-bit audio
#define AUDIO_CHANNELS 1                // Mono audio
#define AUDIO_BUFFER_SIZE 1024          // Increased audio buffer size for stability
#define AUDIO_DMA_BUFFER_COUNT 4        // Increased DMA buffers (was 2, now 4)
#define AUDIO_DMA_BUFFER_SIZE 512       // Increased DMA buffer size (was 256, now 512)

// Audio Processing - Enhanced for M5Stack Atom Echo
#define AUDIO_CAPTURE_INTERVAL_MS 50    // Capture audio every 50ms for better responsiveness
#define AUDIO_TASK_STACK_SIZE 3072      // Increased stack size for I2S processing
#define AUDIO_TASK_PRIORITY 3           // High priority for audio
#define AUDIO_VALIDATION_THRESHOLD 100  // Minimum audio level to consider valid
#define AUDIO_ERROR_RECOVERY_ATTEMPTS 3 // Number of I2S restart attempts

// =============================================================================
// BLE CONFIGURATION - Power optimized for extended battery life
// =============================================================================
#define BLE_MTU_SIZE 517                    // Maximum MTU for efficiency
#define BLE_CHUNK_SIZE 250                  // Safe chunk size for audio transfer (reduced)
#define BLE_AUDIO_TRANSFER_DELAY 5          // Delay for audio transfer
#define BLE_TX_POWER ESP_PWR_LVL_P3         // Higher power for better range

// Power-optimized BLE Advertising
#define BLE_ADV_MIN_INTERVAL 0x0140         // 200ms minimum
#define BLE_ADV_MAX_INTERVAL 0x0280         // 400ms maximum
#define BLE_ADV_TIMEOUT_MS 0                // Never stop advertising
#define BLE_SLEEP_ADV_INTERVAL 45000        // Re-advertise every 45 seconds

// Connection Management
#define BLE_CONNECTION_TIMEOUT_MS 0         // Never timeout connections
#define BLE_TASK_INTERVAL_MS 20000          // 20 second connection check
#define BLE_TASK_STACK_SIZE 1536
#define BLE_TASK_PRIORITY 1

// Connection Parameters
#define BLE_CONN_MIN_INTERVAL 20            // 25ms minimum connection interval
#define BLE_CONN_MAX_INTERVAL 40            // 50ms maximum connection interval
#define BLE_CONN_LATENCY 0                  // No latency for immediate response
#define BLE_CONN_TIMEOUT 800                // 8 second supervision timeout

// =============================================================================
// POWER STATES
// =============================================================================
typedef enum {
    POWER_STATE_ACTIVE,      // Normal operation - audio + BLE active
    POWER_STATE_POWER_SAVE,  // Reduced frequency, longer intervals
    POWER_STATE_LOW_BATTERY, // Minimal operation
    POWER_STATE_SLEEP        // Deep sleep mode
} power_state_t;

// =============================================================================
// TASK CONFIGURATION
// =============================================================================
#define BATTERY_TASK_STACK_SIZE 1024
#define BATTERY_TASK_PRIORITY 1
#define POWER_MANAGEMENT_TASK_STACK_SIZE 1024
#define POWER_MANAGEMENT_TASK_PRIORITY 0

// Status Reporting
#define STATUS_REPORT_INTERVAL_MS 120000    // 2 minutes

// =============================================================================
// BLE UUID DEFINITIONS - OMI Protocol
// =============================================================================
#define OMI_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define AUDIO_DATA_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define AUDIO_CODEC_UUID "19B10002-E8F2-537E-4F6C-D104768A1214"
#define AUDIO_CONTROL_UUID "19B10004-E8F2-537E-4F6C-D104768A1214"

// Battery Service UUID
#define BATTERY_SERVICE_UUID (uint16_t)0x180F
#define BATTERY_LEVEL_UUID (uint16_t)0x2A19

// =============================================================================
// RGB LED CONFIGURATION
// =============================================================================
#define NUM_LEDS 1                      // Single RGB LED
#define LED_BRIGHTNESS 50               // LED brightness (0-255)
#define LED_TYPE NEO_GRB + NEO_KHZ800   // NeoPixel type

// LED Status Colors
#define LED_COLOR_OFF       0x000000    // Black (off)
#define LED_COLOR_BOOT      0x0000FF    // Blue (booting)
#define LED_COLOR_READY     0x00FF00    // Green (ready)
#define LED_COLOR_ACTIVE    0xFF0000    // Red (active/recording)
#define LED_COLOR_LOW_BATT  0xFFFF00    // Yellow (low battery)
#define LED_COLOR_ERROR     0xFF00FF    // Magenta (error)

// =============================================================================
// BUTTON CONFIGURATION
// =============================================================================
#define BUTTON_DEBOUNCE_MS 50             // Button debounce time
#define BUTTON_LONG_PRESS_MS 2000         // Long press duration (2 seconds)
#define BUTTON_DOUBLE_CLICK_MS 500        // Double click detection window

// Button States
typedef enum {
    BUTTON_IDLE,
    BUTTON_PRESSED,
    BUTTON_LONG_PRESS,
    BUTTON_DOUBLE_CLICK,
    BUTTON_RELEASED
} button_state_t;

// LED Status Modes
typedef enum {
    LED_OFF,
    LED_BOOT_SEQUENCE,
    LED_READY,
    LED_RECORDING,
    LED_LOW_BATTERY,
    LED_ERROR,
    LED_SLEEP_MODE
} led_status_t;

// Device Power States
typedef enum {
    DEVICE_BOOTING,
    DEVICE_READY,
    DEVICE_RECORDING,
    DEVICE_POWER_SAVE,
    DEVICE_LOW_BATTERY,
    DEVICE_ERROR,
    DEVICE_SLEEP
} device_state_t;

#endif // CONFIG_H
