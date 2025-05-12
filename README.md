
# 🎯 Crosshair Tracking Camera System

This project is a **real-time visual tracking system** using two microcontrollers:

- 🧠 **ESP32-CAM**: Detects a black crosshair (`+`) on camera input using grayscale image processing and integral image techniques.
- 🤖 **KL25Z (NXP Freedom Board)**: Receives coordinates via UART and uses:
  - An **8x8 LED matrix** to visually indicate the detected position

The system demonstrates **real-time image analysis, UART communication, and actuator control** in an embedded hardware project.

---

## 📦 System Overview

```
        +----------------+      UART      +------------------+
        |                |  ------------> |                  |
        |   ESP32-CAM    |                |     KL25Z        |
        |                | <------------  |                  |
        +----------------+  (Optional)    +------------------+
             ^                                      |
             |                                      |
       MJPEG Streaming                        PWM Servo Control
         + Coordinate                         8x8 LED Matrix Display
           Tracking
```

---

## 🧠 ESP32-CAM (Vision + Detection)

### 📷 Features

- Captures **160×120 grayscale frames**
- Computes **integral image** for efficient box-sum operations
- Detects **dark cross-shaped patterns**
- Outputs detected `(x, y)` coordinates over UART
- Overlays a white cross on the detected position
- Streams MJPEG video over HTTP (browser-compatible)

### 🔧 Configuration

Edit in `setup()`:
```cpp
const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASS";
```

### 📤 Output

- Serial UART: `x,y\n` (coordinates)
- Web stream: `http://<device_ip>/`

---

## 🤖 KL25Z (Control + Feedback)

### 🎮 Features

- **Reads joystick input** via ADC (manual servo control)
- **Receives UART coordinates** from ESP32-CAM (auto mode)
- **Controls two PWM servos** for `X` and `Y` targeting
- **Displays target location** on an **8x8 LED matrix**
- **Mode switching** via a button:
  - `Manual mode`: Joystick input
  - `Locked mode`: Final position

### 🔌 Pin Mapping (KL25Z)

#### Servos
- PTE20 → Servo X (TPM1_CH0)
- PTE21 → Servo Y (TPM1_CH1)

#### Joystick (ADC)
- PTB0 → X axis
- PTB1 → Y axis

#### Button
- PTD3 → Mode toggle (with interrupt)

#### 8x8 LED Matrix
Rows and columns mapped using `gpio_pin_t` structures in code. Refer to `row_pins[]` and `col_pins[]` arrays for exact GPIO configuration.

---

## 🔁 Communication Protocol

- From ESP32-CAM to KL25Z:
  ```
  <x>,<y>\n
  Example: 45,60
  ```
- KL25Z parses this string via `sscanf()` and maps `(x,y)` to:
  - **LED matrix cell**

---

## 🛠️ How to Build

### Flash ESP32-CAM
1. Use **Arduino IDE** or **PlatformIO**
2. Set camera model to `AI-Thinker`
3. Upload the ESP32 sketch
4. Open Serial Monitor (9600 baud) to see IP and UART logs

### Flash KL25Z
1. Use **MCUXpresso** or **mbed** toolchain
2. Flash the C project with UART, ADC, TPM, and GPIO setup
3. Power both boards and connect UART:
   - ESP32-CAM TX → KL25Z RX (via level shifter or voltage divider)

---

## 🔋 Power Supply Tips

- ESP32-CAM can brown out on USB alone. Use external 5V source.
- KL25Z should be USB powered (via debugger port)

---

## 👨‍💻 Author

**Tuna Alatan**  
Electrical and Electronics Engineering, Bilkent University  
GitHub: [@tuna-alatan](https://github.com/tuna-alatan)

---

## 📄 License

MIT License. See `LICENSE` file for more information.
