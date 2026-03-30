// =============================================================================
// Head Impact Monitor — Arduino Nano 33 BLE Rev2
// Revision 3: ADXL375 (SPI) + BMI270 internal gyroscope
//
// This revision uses the Nano 33 BLE Rev2's onboard BMI270 gyroscope
// while the MPU-9250 is unavailable (awaiting header soldering).
// When the MPU-9250 is ready, swap back to Revision 2.
//
// WHAT CHANGED FROM REVISION 2:
//   - Removed Wire.h and all MPU-9250 register-level code
//   - Removed mpu9250_init(), mpu9250_read_gyro(), and related functions
//   - Added Arduino_BMI270_BMM150 library for onboard IMU
//   - Gyro read now calls IMU.readGyroscope() instead of I2C burst read
//   - All signal processing, capture, classification, and serial output
//     logic is identical to Revision 2
//
// HARDWARE:
//   - Arduino Nano 33 BLE Rev2 (onboard BMI270 gyroscope — no wiring needed)
//   - ADXL375 high-g accelerometer (SPI, ±200g)
//   - 3x LEDs: Green (D4), Yellow (D5), Red (D6)
//
// SPI WIRING — ADXL375 → Nano 33 BLE Rev2 (unchanged from Rev 2):
//   VCC  → 3.3V
//   GND  → GND
//   CS   → D10
//   SCL  → D13  (SPI clock)
//   SDA  → D11  (MOSI)
//   SDO  → D12  (MISO)
//
// LED WIRING (unchanged):
//   Green  anode → D4 → 220Ω → GND
//   Yellow anode → D5 → 220Ω → GND
//   Red    anode → D6 → 220Ω → GND
//
// LIBRARY REQUIRED:
//   Arduino_BMI270_BMM150 — install via Arduino IDE Library Manager
//
// SERIAL OUTPUT FORMAT:
//   peak_linear_g,peak_rotational_rad_s2
//   Lines beginning with # are diagnostic messages — Python skips these.
//
// =============================================================================

#include <SPI.h>
#include <Arduino_BMI270_BMM150.h>   // Onboard IMU (BMI270 gyroscope)

// -----------------------------------------------------------------------------
// PIN DEFINITIONS
// -----------------------------------------------------------------------------
#define PIN_LED_GREEN    4
#define PIN_LED_YELLOW   5
#define PIN_LED_RED      6
#define PIN_ADXL375_CS  10

// -----------------------------------------------------------------------------
// ADXL375 REGISTER MAP
// Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.pdf
// -----------------------------------------------------------------------------
#define ADXL375_DEVID          0x00
#define ADXL375_BW_RATE        0x2C
#define ADXL375_POWER_CTL      0x2D
#define ADXL375_DATA_FORMAT    0x31
#define ADXL375_DATAX0         0x32
#define ADXL375_FIFO_CTL       0x38

#define ADXL375_SPI_READ       0x80
#define ADXL375_SPI_MULTIBYTE  0x40

// ADXL375 fixed sensitivity: 49 mg/LSB (±200g, 13-bit, non-configurable)
#define ADXL375_SCALE_G        0.049f

// -----------------------------------------------------------------------------
// SYSTEM CONFIGURATION
// -----------------------------------------------------------------------------

#define SAMPLE_RATE_HZ         500
#define SAMPLE_INTERVAL_US     2000   // 1,000,000 / 500 Hz

// Impact trigger: 10g is well above normal movement but below minor impacts
#define TRIGGER_THRESHOLD_G    10.0f

// Capture window: 20 ms at 500 Hz = 10 samples
// Head impacts typically peak within 10-20 ms
#define CAPTURE_WINDOW_MS      20
#define CAPTURE_SAMPLES        (SAMPLE_RATE_HZ * CAPTURE_WINDOW_MS / 1000)

// Post-impact lockout: prevents re-triggering on the same event
#define LOCKOUT_MS             500

// Low-pass filter coefficient
// At 500 Hz, alpha = 0.3 gives ~30 Hz cutoff
// Lower alpha = more smoothing, higher alpha = faster response
#define LPF_ALPHA              0.3f

// Severity thresholds (two-parameter model)
#define GREEN_LIN_MIN          20.0f
#define YELLOW_LIN_MIN         40.0f
#define YELLOW_LIN_MAX         60.0f
#define YELLOW_ROT_MIN         500.0f
#define YELLOW_ROT_MAX         2500.0f
#define RED_LIN_THRESHOLD      60.0f
#define RED_ROT_THRESHOLD      2500.0f

#define LED_ON_DURATION_MS     2000

// -----------------------------------------------------------------------------
// GLOBAL STATE
// -----------------------------------------------------------------------------

// Low-pass filtered sensor values
float lpf_ax = 0, lpf_ay = 0, lpf_az = 0;  // ADXL375 (g)
float lpf_gx = 0, lpf_gy = 0, lpf_gz = 0;  // BMI270 (rad/s)

// Previous filtered gyro values for numerical differentiation
float prev_gx = 0, prev_gy = 0, prev_gz = 0;
unsigned long prev_sample_us = 0;

// Capture buffer
float buf_lin_g[CAPTURE_SAMPLES];
float buf_rot_rad_s2[CAPTURE_SAMPLES];
int   buf_index = 0;

// State flags
bool capturing        = false;
bool in_lockout       = false;
unsigned long capture_start_ms = 0;
unsigned long lockout_start_ms = 0;

// Hardware status
bool adxl375_ok = false;
bool bmi270_ok  = false;

// -----------------------------------------------------------------------------
// FUNCTION PROTOTYPES
// -----------------------------------------------------------------------------
void    adxl375_init();
bool    adxl375_check_devid();
void    adxl375_write_reg(uint8_t reg, uint8_t val);
uint8_t adxl375_read_reg(uint8_t reg);
void    adxl375_read_accel(float &ax, float &ay, float &az);

void    apply_lpf(float &filtered, float raw, float alpha);
float   vec_magnitude(float x, float y, float z);
void    set_led(int severity);
void    process_capture();
void    classify_and_transmit(float peak_lin_g, float peak_rot_rad_s2);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("# Head Impact Monitor v3 — ADXL375 + BMI270 (internal)");
  Serial.println("# Initializing...");

  // --- LED setup and startup sweep ---
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  set_led(0);
  set_led(1); delay(200);
  set_led(2); delay(200);
  set_led(3); delay(200);
  set_led(0); delay(100);

  // --- ADXL375 via SPI ---
  SPI.begin();
  pinMode(PIN_ADXL375_CS, OUTPUT);
  digitalWrite(PIN_ADXL375_CS, HIGH);
  adxl375_init();

  // --- BMI270 internal gyroscope ---
  // The BMI270_BMM150 library initialises the full onboard IMU.
  // We only call IMU.readGyroscope() — the accelerometer channel
  // is ignored in favour of the higher-range ADXL375.
  if (!IMU.begin()) {
    Serial.println("# ERROR: BMI270 failed to initialise.");
    Serial.println("# This is the onboard sensor — check library installation.");
    Serial.println("# Install: Arduino_BMI270_BMM150 via Library Manager.");
    // Blink red indefinitely to signal hard fault
    while (true) {
      set_led(3); delay(300);
      set_led(0); delay(300);
    }
  }

  bmi270_ok = true;
  Serial.print("# BMI270 gyroscope sample rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  // --- Status report ---
  if (!adxl375_ok) {
    Serial.println("# WARNING: ADXL375 not responding. Check SPI wiring.");
  }

  if (adxl375_ok && bmi270_ok) {
    Serial.println("# All sensors online.");
    Serial.println("# NOTE: Using internal BMI270 gyroscope (±2000 deg/s).");
    Serial.println("# Swap to MPU-9250 after headers are soldered.");
  }

  Serial.println("# Monitoring for impacts...");
  Serial.println("# OUTPUT FORMAT: peak_linear_g,peak_rotational_rad_s2");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  unsigned long now_us = micros();
  unsigned long now_ms = millis();

  // Enforce sample rate
  static unsigned long last_sample_us = 0;
  if (now_us - last_sample_us < SAMPLE_INTERVAL_US) return;
  last_sample_us = now_us;

  // ── 1. Read raw sensor data ────────────────────────────────────────────────

  float raw_ax = 0, raw_ay = 0, raw_az = 0;
  if (adxl375_ok) {
    adxl375_read_accel(raw_ax, raw_ay, raw_az);
  }

  // BMI270 readGyroscope() returns degrees/s — convert to rad/s immediately
  // IMU.gyroscopeAvailable() checks if a new sample is ready from the sensor.
  // If not ready this cycle, we reuse the last filtered value (no change).
  float raw_gx = lpf_gx, raw_gy = lpf_gy, raw_gz = lpf_gz;
  if (bmi270_ok && IMU.gyroscopeAvailable()) {
    float gx_deg, gy_deg, gz_deg;
    IMU.readGyroscope(gx_deg, gy_deg, gz_deg);
    raw_gx = gx_deg * DEG_TO_RAD;
    raw_gy = gy_deg * DEG_TO_RAD;
    raw_gz = gz_deg * DEG_TO_RAD;
  }

  // ── 2. Low-pass filter ────────────────────────────────────────────────────
  apply_lpf(lpf_ax, raw_ax, LPF_ALPHA);
  apply_lpf(lpf_ay, raw_ay, LPF_ALPHA);
  apply_lpf(lpf_az, raw_az, LPF_ALPHA);
  apply_lpf(lpf_gx, raw_gx, LPF_ALPHA);
  apply_lpf(lpf_gy, raw_gy, LPF_ALPHA);
  apply_lpf(lpf_gz, raw_gz, LPF_ALPHA);

  // ── 3. Rotational acceleration (numerical differentiation) ────────────────
  // Differentiate filtered omega to avoid amplifying raw sensor noise.
  // alpha_rot = d(omega)/dt ≈ delta_omega / delta_t
  float rot_accel_rad_s2 = 0;
  if (prev_sample_us > 0) {
    float dt = (now_us - prev_sample_us) * 1e-6f;
    if (dt > 0.0f) {
      float d_gx = (lpf_gx - prev_gx) / dt;
      float d_gy = (lpf_gy - prev_gy) / dt;
      float d_gz = (lpf_gz - prev_gz) / dt;
      rot_accel_rad_s2 = vec_magnitude(d_gx, d_gy, d_gz);
    }
  }
  prev_gx = lpf_gx;
  prev_gy = lpf_gy;
  prev_gz = lpf_gz;
  prev_sample_us = now_us;

  // ── 4. Linear acceleration magnitude ─────────────────────────────────────
  float lin_mag_g = vec_magnitude(lpf_ax, lpf_ay, lpf_az);

  // ── 5. State machine: IDLE → CAPTURING → PROCESSING → LOCKOUT ─────────────

  if (in_lockout) {
    if (now_ms - lockout_start_ms >= LOCKOUT_MS) {
      in_lockout = false;
      Serial.println("# Ready.");
    }
    return;
  }

  if (capturing) {
    if (buf_index < CAPTURE_SAMPLES && (now_ms - capture_start_ms) < CAPTURE_WINDOW_MS) {
      buf_lin_g[buf_index]      = lin_mag_g;
      buf_rot_rad_s2[buf_index] = rot_accel_rad_s2;
      buf_index++;
    } else {
      capturing        = false;
      process_capture();
      in_lockout       = true;
      lockout_start_ms = now_ms;
    }

  } else {
    // IDLE — watch for trigger
    if (adxl375_ok && lin_mag_g > TRIGGER_THRESHOLD_G) {
      Serial.print("# Triggered: ");
      Serial.print(lin_mag_g, 1);
      Serial.println(" g — capturing...");

      capturing        = true;
      capture_start_ms = now_ms;
      buf_index        = 0;

      // Seed with triggering sample so leading edge isn't missed
      buf_lin_g[0]      = lin_mag_g;
      buf_rot_rad_s2[0] = rot_accel_rad_s2;
      buf_index         = 1;
    }
  }
}

// =============================================================================
// PROCESS CAPTURE BUFFER
// =============================================================================
void process_capture() {
  if (buf_index == 0) return;

  float peak_lin_g      = 0;
  float peak_rot_rad_s2 = 0;

  for (int i = 0; i < buf_index; i++) {
    if (buf_lin_g[i]      > peak_lin_g)      peak_lin_g      = buf_lin_g[i];
    if (buf_rot_rad_s2[i] > peak_rot_rad_s2) peak_rot_rad_s2 = buf_rot_rad_s2[i];
  }

  Serial.print("# Capture done. n=");
  Serial.print(buf_index);
  Serial.print("  Lin=");
  Serial.print(peak_lin_g, 1);
  Serial.print("g  Rot=");
  Serial.print(peak_rot_rad_s2, 1);
  Serial.println(" rad/s2");

  classify_and_transmit(peak_lin_g, peak_rot_rad_s2);
}

// =============================================================================
// CLASSIFY AND TRANSMIT
// =============================================================================
void classify_and_transmit(float peak_lin_g, float peak_rot_rad_s2) {
  int severity = 0;

  if (peak_lin_g > RED_LIN_THRESHOLD || peak_rot_rad_s2 > RED_ROT_THRESHOLD) {
    severity = 3;
  } else if ((peak_lin_g      >= YELLOW_LIN_MIN && peak_lin_g      <= YELLOW_LIN_MAX) ||
             (peak_rot_rad_s2 >= YELLOW_ROT_MIN && peak_rot_rad_s2 <= YELLOW_ROT_MAX)) {
    severity = 2;
  } else if (peak_lin_g >= GREEN_LIN_MIN) {
    severity = 1;
  }

  set_led(severity);

  // Data line — parsed by Python dashboard (no # prefix)
  Serial.print("Severity (G1, Y2, R3): ");
  Serial.println(severity,1);
  
  Serial.print(peak_lin_g, 2);
  Serial.print(",");
  Serial.println(peak_rot_rad_s2, 2);

  delay(LED_ON_DURATION_MS);
  set_led(0);
}

// =============================================================================
// UTILITY: Exponential Moving Average Low-Pass Filter
// =============================================================================
void apply_lpf(float &filtered, float raw, float alpha) {
  filtered = alpha * raw + (1.0f - alpha) * filtered;
}

// =============================================================================
// UTILITY: 3D Vector Magnitude
// =============================================================================
float vec_magnitude(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

// =============================================================================
// UTILITY: LED Control
// severity: 0=off, 1=green, 2=yellow, 3=red
// =============================================================================
void set_led(int severity) {
  digitalWrite(PIN_LED_GREEN,  severity == 1 ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, severity == 2 ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,    severity == 3 ? HIGH : LOW);
}

// =============================================================================
// ADXL375 — INITIALIZATION
// =============================================================================
void adxl375_init() {
  adxl375_ok = adxl375_check_devid();
  if (!adxl375_ok) return;

  // DATA_FORMAT: full resolution, ±200g range bits set to 11 (required by ADXL375)
  adxl375_write_reg(ADXL375_DATA_FORMAT, 0x0B);

  // BW_RATE: 400 Hz output data rate
  adxl375_write_reg(ADXL375_BW_RATE, 0x0C);

  // FIFO_CTL: bypass mode (read directly, no FIFO buffering)
  adxl375_write_reg(ADXL375_FIFO_CTL, 0x00);

  // POWER_CTL: enter measurement mode
  adxl375_write_reg(ADXL375_POWER_CTL, 0x08);

  delay(10);
  Serial.println("# ADXL375 initialized: +-200g, 400 Hz ODR, 13-bit full res");
}

// =============================================================================
// ADXL375 — DEVICE ID CHECK (expected: 0xE5)
// =============================================================================
bool adxl375_check_devid() {
  uint8_t id = adxl375_read_reg(ADXL375_DEVID);
  Serial.print("# ADXL375 DEVID: 0x");
  Serial.println(id, HEX);
  return (id == 0xE5);
}

// =============================================================================
// ADXL375 — SPI WRITE
// ADXL375 uses SPI Mode 3 (CPOL=1, CPHA=1)
// =============================================================================
void adxl375_write_reg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL375_CS, LOW);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(val);
  digitalWrite(PIN_ADXL375_CS, HIGH);
  SPI.endTransaction();
}

// =============================================================================
// ADXL375 — SPI READ
// =============================================================================
uint8_t adxl375_read_reg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL375_CS, LOW);
  SPI.transfer(reg | ADXL375_SPI_READ);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_ADXL375_CS, HIGH);
  SPI.endTransaction();
  return val;
}

// =============================================================================
// ADXL375 — BURST READ ALL AXES (6 bytes)
// Register order: DATAX0 (XL), DATAX1 (XH), YL, YH, ZL, ZH
// =============================================================================
void adxl375_read_accel(float &ax, float &ay, float &az) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL375_CS, LOW);
  SPI.transfer(ADXL375_DATAX0 | ADXL375_SPI_READ | ADXL375_SPI_MULTIBYTE);

  uint8_t xl = SPI.transfer(0x00);
  uint8_t xh = SPI.transfer(0x00);
  uint8_t yl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00);
  uint8_t zl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00);

  digitalWrite(PIN_ADXL375_CS, HIGH);
  SPI.endTransaction();

  // Right-justified 13-bit two's complement → signed 16-bit → scale to g
  int16_t raw_x = (int16_t)((xh << 8) | xl);
  int16_t raw_y = (int16_t)((yh << 8) | yl);
  int16_t raw_z = (int16_t)((zh << 8) | zl);

  ax = raw_x * ADXL375_SCALE_G;
  ay = raw_y * ADXL375_SCALE_G;
  az = raw_z * ADXL375_SCALE_G;
}
