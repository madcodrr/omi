# OMI M5Stack Atom Echo Firmware

This firmware enables the M5Stack Atom Echo device to work with the OMI ecosystem, providing audio capture and BLE communication capabilities.

## Hardware Requirements

- **M5Stack Atom Echo** with ESP32-PICO chip
- Built-in I2S microphone and speaker
- RGB LED (NeoPixel)
- Physical button

## Pin Configuration

The firmware is configured for the M5Stack Atom Echo pin layout:

| Function | GPIO Pin | Description |
|----------|----------|-------------|
| I2S DOUT | G22 | Speaker Data Out (SPK-I2S) |
| I2S BCLK | G19 | Bit Clock (SPK-I2S) |
| I2S DIN  | G23 | Microphone Data In |
| I2S LRC  | G33 | Left/Right Clock (Word Select) |
| Button   | G39 | Physical button with interrupt |
| RGB LED  | G27 | Status indicator (NeoPixel) |

## Features

### 🎤 Audio Capture
- 16kHz sample rate, 16-bit mono audio
- Real-time I2S microphone input
- BLE streaming to connected devices
- Automatic gain control

### 🔊 Audio Playback
- I2S speaker output capability
- Support for audio feedback and notifications

### 📶 Connectivity
- **Bluetooth LE**: OMI protocol support
- **WiFi**: ESP32-PICO WiFi capabilities (ready for future features)
- Automatic device discovery and pairing

### 🎛️ User Interface
- **Button Controls**:
  - Short press: Toggle audio recording
  - Long press (2s): Toggle power save mode
- **LED Status Indicators**:
  - Blue (blinking): Booting/initializing
  - Green (solid): Ready for connection
  - Red (solid): Recording audio
  - Yellow (blinking): Low battery warning
  - Magenta (blinking): Error state
  - Blue (dim, slow): Sleep mode

### ⚡ Power Management
- Dynamic CPU frequency scaling
- Light sleep mode during idle periods
- Power save mode for extended operation
- Battery status reporting via BLE

## Building and Flashing

### Prerequisites

1. **PlatformIO** installed in VS Code or as CLI
2. **ESP32 development environment** set up
3. **M5Stack Atom Echo** device

### Build Steps

1. Clone the repository and navigate to the firmware directory:
   ```bash
   cd omiGlass/firmware-m5stack-atom-echo/
   ```

2. Install dependencies:
   ```bash
   pio lib install
   ```

3. Build the firmware:
   ```bash
   pio run -e m5stack_atom_echo
   ```

4. Flash to device:
   ```bash
   pio run -e m5stack_atom_echo -t upload
   ```

### Monitor Serial Output

```bash
pio device monitor -b 115200
```

## BLE Protocol

The firmware implements the OMI BLE protocol with the following services:

### OMI Service (`19B10000-E8F2-537E-4F6C-D104768A1214`)
- **Audio Data** (`19B10001-E8F2-537E-4F6C-D104768A1214`): Streaming audio data
- **Audio Control** (`19B10002-E8F2-537E-4F6C-D104768A1214`): Control commands

### Battery Service (`0x180F`)
- **Battery Level** (`0x2A19`): Current battery percentage

### Device Information Service (`0x180A`)
- Manufacturer name, model, firmware version, hardware revision

## Audio Control Commands

Send these values to the Audio Control characteristic:

| Command | Value | Description |
|---------|-------|-------------|
| Start Recording | `1` | Begin audio capture |
| Stop Recording | `0` | Stop audio capture |
| Toggle Recording | `2` | Toggle recording state |
| Enter Power Save | `10` | Reduce power consumption |
| Exit Power Save | `11` | Resume normal operation |

## Configuration

Key configuration options in `src/config.h`:

```cpp
// Audio Settings
#define AUDIO_SAMPLE_RATE 16000     // 16kHz sample rate
#define AUDIO_BITS_PER_SAMPLE 16    // 16-bit audio
#define AUDIO_CHANNELS 1            // Mono audio

// Power Management
#define MAX_CPU_FREQ_MHZ 240        // Maximum CPU frequency
#define MIN_CPU_FREQ_MHZ 80         // Minimum for stable I2S
#define IDLE_THRESHOLD_MS 45000     // Power save after 45s idle

// LED Settings
#define LED_BRIGHTNESS 50           // RGB LED brightness (0-255)
```

## Troubleshooting

### Common Issues

1. **No audio capture**: Check I2S pin connections and microphone power
2. **BLE not advertising**: Restart device, check for BLE conflicts
3. **LED not working**: Verify RGB LED pin (G27) and power supply
4. **Button not responsive**: Check GPIO39 connection and pull-up resistor

### Debug Output

Enable debug output by setting `CORE_DEBUG_LEVEL=5` in `platformio.ini`:

```ini
build_flags = 
    -DCORE_DEBUG_LEVEL=5
```

### Serial Monitor Commands

Monitor the serial output for debug information:
- I2S initialization status
- BLE connection events
- Audio buffer statistics
- Power management state changes

## Development

### Code Structure

```
src/
├── main.cpp          # Arduino main functions
├── app.h             # Function declarations
├── app.cpp           # Main application logic
└── config.h          # Hardware and feature configuration
```

### Key Functions

- `setupAudio()`: Initialize I2S audio system
- `setupBLE()`: Configure BLE services and advertising
- `processAudioData()`: Handle real-time audio capture
- `handleButton()`: Process button press events
- `updateLED()`: Manage RGB LED status display

## Future Enhancements

- [ ] Audio compression for BLE efficiency
- [ ] Voice activity detection (VAD)
- [ ] WiFi-based audio streaming
- [ ] Over-the-air (OTA) firmware updates
- [ ] Advanced power management
- [ ] Audio quality optimization

## License

This firmware is part of the OMI project. See the main repository for license information.

## Support

For support and questions:
- Open an issue in the main OMI repository
- Check the OMI documentation
- Join the OMI community discussions
