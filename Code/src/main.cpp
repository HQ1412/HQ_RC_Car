#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_MAX1704X.h>

// J6-J3 FR BR BL FL

// sets the access point credentials
const char* ssid = "HQ RC Car";
const char* password = "Vr00m3^2";
const char* mDNS = "hrccp";

// sets the web server port number to 80
WiFiServer server(80);

// buzzer is on GPIO 32
const int BUZZER_PIN = 32;

// motor pins
const int FR_MODE_PIN = 23;
const int FR_PH_PIN = 25;
const int FR_EN_PIN = 26;

const int BR_MODE_PIN = 19;
const int BR_PH_PIN = 21;
const int BR_EN_PIN = 22;

const int BL_MODE_PIN = 16;
const int BL_PH_PIN = 17;
const int BL_EN_PIN = 18;

const int FL_MODE_PIN = 13;
const int FL_PH_PIN = 14;
const int FL_EN_PIN = 15;

// Buck Boost Power Good pin
const int PG_PIN = 39;

// MXC400xXC I2C pins (named for accel so they are clearly associated)
const int ACCEL_I2C_SDA_PIN = 5; // SDA on GPIO5 (accel)
const int ACCEL_I2C_SCL_PIN = 4; // SCL on GPIO4 (accel)

// Accel axis sign: set to -1 because front of car is -X; set to +1 if you mount differently
const int ACCEL_X_SIGN = -1;

// Battery device name (for prints / identification). Uses same I2C pins as accel.
const char* BATTERY_I2C_NAME = "MAX17048";

// MXC400xXC registers
const uint8_t REG_XOUT_U = 0x03;
const uint8_t REG_XOUT_L = 0x04;
const uint8_t REG_CONTROL = 0x0D; // contains FSR bits

// Addresses 0x10..0x17 selectable by hardware (per datasheet)
const uint8_t ADDR_BASE = 0x10;

// Integration & calibration params
const int CALIB_SAMPLES = 200;
const float G_TO_MS2 = 9.80665f;
const float SENSITIVITY_LSB_PER_G = 1024.0f; // default for +/-2g (change if you set FSR)
const unsigned long SAMPLE_INTERVAL_US = 10000; // 100 Hz => 10,000 us

// Behavior flags / tuning (change these as needed)
const bool DEBUG_MODE = false; // set true to print raw/sample/debug columns
// If you already know the sensor address and it's fixed, set KNOWN_I2C_ADDR to that
// (e.g. 0x15). Set to 0 to enable auto-scan.
const uint8_t KNOWN_I2C_ADDR = 0x15;

// ZUPT (zero-velocity update) tuning
const bool ZUPT_ENABLED = true;
const int ZUPT_WINDOW = 50; // number of recent samples to consider (~window * dt)
const float ZUPT_STDDEV_THRESHOLD_G = 0.02f; // if accel stddev (g) below this -> stationary
const float ZUPT_HOLD_TIME_S = 0.3f; // require this many seconds below threshold
const float BIAS_ADAPT_BETA = 0.02f; // how quickly to adapt bias while stationary

// Safety clamps
const float MAX_STEP_DV = 2.0f; // m/s maximum delta-v per step (protects against spikes)
const float MAX_DT_S = 0.050f; // ignore/skip samples where dt > 50 ms

// runtime state
int8_t i2c_addr = -1;
float bias_g = 0.0f;
float vel_m_s = 0.0f;
float last_acc_ms2 = 0.0f;
unsigned long last_sample_us = 0;

// ----- Battery gauge state -----
Adafruit_MAX17048 maxlipo;
bool battery_present = false;
unsigned long last_batt_ms = 0;
const unsigned long BATTERY_POLL_MS = 2000;

unsigned long last_PG_ms = 0;
const unsigned long PG_POLL_MS = 2000;

// ZUPT rolling buffer state
float zupt_buffer[ZUPT_WINDOW];
int zupt_idx = 0;
int zupt_count = 0;
float zupt_sum = 0.0f;
float zupt_sumsq = 0.0f;
unsigned long zupt_below_since_us = 0;

// Helper: read one register byte, return true if ok
bool i2cReadByte(uint8_t addr, uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)addr, 1);
  if (Wire.available() < 1) return false;
  out = Wire.read();
  return true;
}

// Read 12-bit XOUT (two registers). Returns true and fills raw if ok.
bool readRawX(int16_t &rawOut) {
  uint8_t hi, lo;
  if (!i2cReadByte(i2c_addr, REG_XOUT_U, hi)) return false;
  if (!i2cReadByte(i2c_addr, REG_XOUT_L, lo)) return false;
  uint16_t raw12 = ((uint16_t)hi << 4) | ((uint16_t)(lo >> 4) & 0x0F);
  // sign-extend 12-bit
  if (raw12 & 0x800) rawOut = (int16_t)(raw12 | 0xF000);
  else rawOut = (int16_t)raw12;
  return true;
}

// Find MXC400xXC on bus (try 0x10..0x17 first), returns -1 if not found
int8_t findSensorAddress() {
  for (uint8_t a = 0; a < 8; ++a) {
    uint8_t addr = ADDR_BASE + a;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return (int8_t)addr;
  }
  // fallback: full scan
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return (int8_t)addr;
  }
  return -1;
}

// Calibrate bias (stationary). Reads multiple raw samples and computes bias in g.
void calibrateBias() {
  long sum = 0;
  int valid = 0;
  for (int i = 0; i < CALIB_SAMPLES; ++i) {
    int16_t raw;
    if (readRawX(raw)) {
      sum += raw;
      valid++;
    }
    delay(5);
  }
  if (valid > 0) {
    float avgRaw = (float)sum / (float)valid;
    // store bias in signed (vehicle) frame so sign flips are consistent
    bias_g = (ACCEL_X_SIGN * avgRaw) / SENSITIVITY_LSB_PER_G;
  } else {
    bias_g = 0.0f;
  }
}

// Forward declarations for accel abstraction (defined below)
void accelInit();
void accelLoop();
// finer-grained accel helpers
bool accelReadRaw(int16_t &rawOut);
float accelRawToSignedG(int16_t raw);
void accelZuptUpdate(float ax_g, unsigned long now, bool &out_is_stationary);
void accelIntegrate(float ax_ms2, float dt);

// Battery gauge helpers
void batteryInit();
void batteryLoop();

void PGLoop();

void setup() {

  // initialize serial 
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("Serial started at 115200");

  // starts the Access Point
  WiFi.softAP(ssid, password);

  // Sets the IP address of the Access Point
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();

  // Initialize mDNS (best-effort)
  if (MDNS.begin(mDNS)) {
    Serial.print("mDNS: ");
    Serial.println(String(mDNS) + ".local");
  } else {
    Serial.println("mDNS begin failed");
  }
  
  // sets the BUZZER_PIN as an output
  pinMode(BUZZER_PIN, OUTPUT);


  // test FR motor
  pinMode(FR_MODE_PIN, OUTPUT);
  pinMode(FR_PH_PIN, OUTPUT);
  pinMode(FR_EN_PIN, OUTPUT);
  digitalWrite(FR_MODE_PIN, HIGH); // set to PH/EN mode
  digitalWrite(FR_PH_PIN, HIGH);   // set direction
  analogWrite(FR_EN_PIN, 255);     // set speed (0-255)

  // test BR motor
  pinMode(BR_MODE_PIN, OUTPUT);
  pinMode(BR_PH_PIN, OUTPUT);
  pinMode(BR_EN_PIN, OUTPUT);
  digitalWrite(BR_MODE_PIN, HIGH); // set to PH/EN mode
  digitalWrite(BR_PH_PIN, HIGH);   // set direction
  analogWrite(BR_EN_PIN, 255);     // set speed (0-255)

  // test BL motor
  pinMode(BL_MODE_PIN, OUTPUT);
  pinMode(BL_PH_PIN, OUTPUT);
  pinMode(BL_EN_PIN, OUTPUT);
  digitalWrite(BL_MODE_PIN, HIGH); // set to PH/EN mode
  digitalWrite(BL_PH_PIN, HIGH);   // set direction
  analogWrite(BL_EN_PIN, 255);     // set speed (0-255)

  // test FL motor
  pinMode(FL_MODE_PIN, OUTPUT);
  pinMode(FL_PH_PIN, OUTPUT);
  pinMode(FL_EN_PIN, OUTPUT);
  digitalWrite(FL_MODE_PIN, HIGH); // set to PH/EN mode
  digitalWrite(FL_PH_PIN, HIGH);   // set direction
  analogWrite(FL_EN_PIN, 255);     // set speed (0-255)

  // sets the PG_PIN as an input
  pinMode(PG_PIN, INPUT);

  boolean PG = digitalRead(PG_PIN);

  // initialize accelerometer (I2C, address, calibration)
  accelInit();

  // initialize battery gauge (reuses Wire started by accelInit)
  batteryInit();
}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    delay(50);
  }

  // accel sampling handled in accelLoop()
  accelLoop();

  // battery polling handled separately (non-blocking)
  batteryLoop();

  PGLoop();

  // short idle so other tasks (WiFi stack) can run; keep small to allow sampling
  delay(1);

  //tone(BUZZER_PIN, 2000, 1000);
  //delay(1000);
}

void accelInit() {
  // start I2C on accel pins
  Wire.begin(ACCEL_I2C_SDA_PIN, ACCEL_I2C_SCL_PIN);
  delay(10);

  // If provided a known address, use it; otherwise scan the bus
  if (KNOWN_I2C_ADDR != 0) {
    i2c_addr = (int8_t)KNOWN_I2C_ADDR;
    Serial.print("Using known MXC400xXC I2C addr 0x"); Serial.println(i2c_addr, HEX);
  } else {
    int8_t found = findSensorAddress();
    if (found < 0) {
      Serial.println("MXC400xXC not found on I2C bus. Run wiring/check address.");
      i2c_addr = -1;
    } else {
      i2c_addr = found;
      Serial.print("MXC400xXC found at 0x"); Serial.println(i2c_addr, HEX);
    }
  }

  if (i2c_addr >= 0) {
    // calibrate bias while stationary
    calibrateBias();
    Serial.print("Bias g: "); Serial.println(bias_g, 6);
    last_sample_us = micros();
  }
}

bool accelReadRaw(int16_t &rawOut) {
  return readRawX(rawOut);
}

float accelRawToSignedG(int16_t raw) {
  float raw_g = (float)raw / SENSITIVITY_LSB_PER_G;
  return (float)ACCEL_X_SIGN * raw_g;
}

void accelZuptUpdate(float ax_g, unsigned long now, bool &out_is_stationary) {
  out_is_stationary = false;
  if (!ZUPT_ENABLED) return;
  float old = 0.0f;
  if (zupt_count == ZUPT_WINDOW) {
    old = zupt_buffer[zupt_idx];
    zupt_sum -= old;
    zupt_sumsq -= old * old;
  } else {
    zupt_count++;
  }
  zupt_buffer[zupt_idx] = ax_g;
  zupt_sum += ax_g;
  zupt_sumsq += ax_g * ax_g;
  zupt_idx = (zupt_idx + 1) % ZUPT_WINDOW;

  if (zupt_count == ZUPT_WINDOW) {
    float mean = zupt_sum / (float)zupt_count;
    float var = (zupt_sumsq / (float)zupt_count) - (mean * mean);
    if (var < 0) var = 0;
    float stddev = sqrtf(var);
    if (stddev < ZUPT_STDDEV_THRESHOLD_G) {
      if (zupt_below_since_us == 0) zupt_below_since_us = now;
      else if ((now - zupt_below_since_us) >= (unsigned long)(ZUPT_HOLD_TIME_S * 1e6f)) {
        out_is_stationary = true;
        bias_g += BIAS_ADAPT_BETA * mean;
      }
    } else {
      zupt_below_since_us = 0;
    }
  }
}

void accelIntegrate(float ax_ms2, float dt) {
  float delta_v = 0.5f * (ax_ms2 + last_acc_ms2) * dt;
  if (fabsf(delta_v) > MAX_STEP_DV) {
    delta_v = (delta_v > 0 ? 1 : -1) * MAX_STEP_DV;
  }
  vel_m_s += delta_v;
  last_acc_ms2 = ax_ms2;
}

void accelLoop() {
  if (i2c_addr < 0) return;
  unsigned long now = micros();
  unsigned long dt_us = now - last_sample_us;
  if (dt_us >= SAMPLE_INTERVAL_US) {
    float dt = dt_us * 1e-6f;
    // protect against very large dt (loop stalls) — skip sample if too large
    if (dt > MAX_DT_S) {
      if (DEBUG_MODE) Serial.println("Skipping sample: dt too large");
      last_sample_us = now;
      // reset last_acc to avoid a huge trapezoid next time
      last_acc_ms2 = 0.0f;
    } else {
      last_sample_us = now;
      int16_t raw;
      if (accelReadRaw(raw)) {
        // convert to signed g in vehicle frame and remove bias
        float signed_raw_g = accelRawToSignedG(raw);
        float ax_g = signed_raw_g - bias_g;
        float ax_ms2 = ax_g * G_TO_MS2;

        // ZUPT update (may adapt bias and indicate stationary)
        bool is_stationary = false;
        accelZuptUpdate(ax_g, now, is_stationary);

        // integrate acceleration into velocity
        accelIntegrate(ax_ms2, dt);

        if (is_stationary) {
          // zero velocity and reset integrator state when stationary
          vel_m_s = 0.0f;
          last_acc_ms2 = 0.0f;
          if (DEBUG_MODE) Serial.println("ZUPT: stationary detected, vel=0, bias adapted");
        }

        // simple high-pass drift mitigation (running average) for printed value
        static float vel_avg = 0.0f;
        const float alpha = 0.995f;
        vel_avg = alpha * vel_avg + (1.0f - alpha) * vel_m_s;
        float vel_hp = vel_m_s - vel_avg;

        if (DEBUG_MODE) {
          // print columns: raw, raw_g, ax_g, ax_ms2, dt, mean, stddev, vel, vel_hp
          float mean = (zupt_count > 0) ? (zupt_sum / (float)zupt_count) : 0.0f;
          float var = (zupt_count > 0) ? (zupt_sumsq / (float)zupt_count) - (mean * mean) : 0.0f;
          if (var < 0) var = 0;
          float stddev = sqrtf(var);
          float raw_g = (float)raw / SENSITIVITY_LSB_PER_G;
          Serial.print(raw); Serial.print(',');
          Serial.print(raw_g, 4); Serial.print(',');
          Serial.print(ax_g, 6); Serial.print(',');
          Serial.print(ax_ms2, 4); Serial.print(',');
          Serial.print(dt, 6); Serial.print(',');
          Serial.print(mean, 6); Serial.print(',');
          Serial.print(stddev, 6); Serial.print(',');
          Serial.print(vel_m_s, 4); Serial.print(',');
          Serial.println(vel_hp, 4);
        } else {
          Serial.print("vel(m/s): "); Serial.println(vel_hp, 4);
        }
      } else {
        Serial.println("Failed to read MXC400xXC X register");
      }
    }
  }
}

void batteryInit() {
  // Wire should already be initialized by accelInit(); try to start the sensor
  battery_present = false;
  if (maxlipo.begin(&Wire)) {
    battery_present = true;
    Serial.print("Found "); Serial.print(BATTERY_I2C_NAME); Serial.print(" Chip ID: 0x");
    Serial.println(maxlipo.getChipID(), HEX);
  } else {
    Serial.print(BATTERY_I2C_NAME); Serial.println(" not found on I2C bus (check battery). Will retry periodically.");
    battery_present = false;
  }
  last_batt_ms = millis();
}

void batteryLoop() {
  unsigned long now = millis();
  if ((now - last_batt_ms) < BATTERY_POLL_MS) return;
  last_batt_ms = now;

  if (!battery_present) {
    // try to initialize periodically
    if (maxlipo.begin(&Wire)) {
      battery_present = true;
      Serial.print("Found "); Serial.print(BATTERY_I2C_NAME); Serial.print(" Chip ID: 0x");
      Serial.println(maxlipo.getChipID(), HEX);
    } else {
      // still not present; skip
      return;
    }
  }

  float cellVoltage = maxlipo.cellVoltage();
  float cellPercent = maxlipo.cellPercent();
  if (isnan(cellVoltage) || isnan(cellPercent)) {
    Serial.println("Failed to read battery gauge (no battery or gauge hibernating)");
    return;
  }
  Serial.print("battery(V): "); Serial.println(cellVoltage, 3);
  Serial.print("battery(%): "); Serial.println(cellPercent, 1);
}

void PGLoop() {
  unsigned long now = millis();
  if ((now - last_PG_ms) < PG_POLL_MS) return;
  last_PG_ms = now;
  if (boolean PG = HIGH) {
    Serial.println("Power Good!");
  } else {
    Serial.println("Power Bad!");
  }
}