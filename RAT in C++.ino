#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <LSM303.h>
#include <Preferences.h>

// OLED display dimensions for 0.91" SSD1306 (128x32)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1  // Reset pin (not used, tie to reset line or use -1 if none)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2C addresses (change if needed)
#define BME280_I2C_ADDR_PRIMARY   0x76
#define BME280_I2C_ADDR_SECONDARY 0x77

Adafruit_BME280 bme;
LSM303 compass;
Preferences prefs;

// Pin definitions for buttons (GPIO on ESP32)
#define PIN_BTN_UP      13
#define PIN_BTN_DOWN    12
#define PIN_BTN_LEFT    14
#define PIN_BTN_RIGHT   27
#define PIN_BTN_CENTER  26  // joystick center (select)
#define PIN_BTN_BACK    25  // back/cancel
#define PIN_BTN_MODE    33  // mode toggle (manual/continuous or hold for calibration)
#define PIN_BTN_TRIGGER 32  // external trigger button for rangefinder

// UART for rangefinder
HardwareSerial RangeSerial(2);
#define PIN_RANGE_TX 17  // TX to rangefinder (if needed)
#define PIN_RANGE_RX 16  // RX from rangefinder

// Bullet data structure
struct Bullet {
  const char* code;      // short code/key
  const char* name;      // full name (if needed)
  double bc;
  double muzzle_velocity; // m/s
  double weight_gr;       // in grains
  double sight_height;    // in meters (sight height above bore)
  double diameter;        // in meters (bullet diameter)
  char drag_model;        // 'G1' or 'G7'
};

// Bullet profiles (all bullet types from original script)
static const Bullet bulletList[] = {
  { "308_M80",    ".308 M80 Ball",          0.400, 840.0, 147.0, 0.06985, 0.00782, 'G' /* will interpret 'G' + number below */ },
  { "308_SUB",    ".308 Subsonic",          0.518, 325.0, 200.0, 0.06985, 0.00782, 'G' },
  { "300BLK_SUP", ".300 Blackout Supersonic",0.305, 670.0, 125.0, 0.06985, 0.00782, 'G' },
  { "300BLK_SUB", ".300 Blackout Subsonic", 0.660, 305.0, 220.0, 0.06985, 0.00782, 'G' },
  { "556_M193",   "5.56 NATO 55gr M193",    0.243, 990.0,  55.0, 0.06985, 0.00570, 'G' },
  { "68BLK_SUP",  "6.8 Blackout Supersonic",0.370, 780.0, 110.0, 0.06985, 0.00680, 'G' },
  { "68BLK_SUB",  "6.8 Blackout Subsonic",  0.450, 330.0, 180.0, 0.06985, 0.00680, 'G' }
};
static const int NUM_BULLETS = sizeof(bulletList) / sizeof(Bullet);

// Drag tables (ft/s, Cd) for G1 and G7 models
struct DragEntry { int vel_fps; double Cd; };
static const DragEntry G1_TABLE[] = {
  {137, 0.262}, {228, 0.230}, {274, 0.211}, {366, 0.203}, {457, 0.198},
  {549, 0.195}, {640, 0.192}, {732, 0.188}, {823, 0.185}, {914, 0.182},
  {1006,0.179}, {1097,0.176}, {1189,0.172}, {1280,0.169}, {1372,0.165},
  {1463,0.162}, {1555,0.159}, {1646,0.156}, {1738,0.152}, {1829,0.149},
  {1920,0.147}, {2012,0.144}, {2103,0.141}
};
static const DragEntry G7_TABLE[] = {
  {137, 0.120}, {228, 0.112}, {274, 0.107}, {366, 0.104}, {457, 0.102},
  {549, 0.100}, {640, 0.098}, {732, 0.095}, {823, 0.093}, {914, 0.091},
  {1006,0.089}, {1097,0.087}, {1189,0.085}, {1280,0.083}, {1372,0.081},
  {1463,0.079}, {1555,0.077}, {1646,0.075}, {1738,0.073}, {1829,0.071},
  {1920,0.069}, {2012,0.067}, {2103,0.065}
};
static const int G1_COUNT = sizeof(G1_TABLE)/sizeof(DragEntry);
static const int G7_COUNT = sizeof(G7_TABLE)/sizeof(DragEntry);

// Global state variables
int currentBulletIndex = 0;
bool continuousMode = false;
bool calibrating = false;
double windSpeed = 0.0;     // m/s
double windDirDeg = 0.0;    // wind coming from this compass direction (0-360)
double lastRange = 0.0;     // last measured range (m)
bool haveRange = false;
char lastSolutionStr[16] = "";  // last displayed solution string for comparison
bool editing = false;
int selectedField = 0;  // 1=Bullet, 2=Wind Speed, 3=Wind Direction (only active when editing)

// Button state tracking for debouncing/edge detect
bool lastBtnUp = false, lastBtnDown = false, lastBtnLeft = false, lastBtnRight = false;
bool lastBtnCenter = false, lastBtnBack = false, lastBtnMode = false, lastBtnTrigger = false;
unsigned long modePressStart = 0;
bool modeHoldHandled = false;

// Magnetometer calibration values
LSM303::vector<int16_t> magMin, magMax;
bool magCalibrated = false;

// Utility functions for ballistic calculations
const double R_AIR = 287.05;  // specific gas constant for dry air (J/kg·K)
double speedOfSound(double tempC) {
  // Speed of sound in air (approx) = sqrt(gamma * R_air * T(K)) with gamma ~1.4
  return sqrt(1.4 * R_AIR * (tempC + 273.15));
}
double airDensity(double tempC, double pressure_hPa, double humidity) {
  double T = tempC + 273.15;
  double P = pressure_hPa * 100.0;  // convert hPa to Pa
  // saturation vapor pressure (hPa) over water at tempC
  double saturation = 6.1078 * pow(10.0, (7.5 * tempC) / (tempC + 237.3));
  double vaporPressure = (humidity / 100.0) * saturation * 100.0;  // convert to Pa
  double dryAirPressure = P - vaporPressure;
  return dryAirPressure / (R_AIR * T);
}
double interpolateCd(double velocity_mps, const DragEntry* table, int tableSize) {
  // velocity in m/s, table velocities in ft/s
  double v_fps = velocity_mps * 3.28084;
  for (int i = 0; i < tableSize - 1; ++i) {
    if (v_fps >= table[i].vel_fps && v_fps <= table[i+1].vel_fps) {
      double v1 = table[i].vel_fps;
      double v2 = table[i+1].vel_fps;
      double cd1 = table[i].Cd;
      double cd2 = table[i+1].Cd;
      // Linear interpolation between table points
      return cd1 + (cd2 - cd1) * ((v_fps - v1) / (v2 - v1));
    }
  }
  // if velocity beyond last table entry, return last Cd value
  return table[tableSize - 1].Cd;
}
double transonicCorrection(double cd, double mach) {
  if (mach >= 0.8 && mach <= 1.2) {
    return cd * 1.15;
  }
  return cd;
}

// Compute ballistic trajectory and return drop/drift in mils
void computeBallisticSolution(double range_m, double pitchRad, double headingDeg,
                              double windSpeed_mps, double windDirDeg,
                              double& outDropMils, double& outDriftMils) {
  if (range_m <= 0) {
    // no valid range, output zeros
    outDropMils = 0.0;
    outDriftMils = 0.0;
    return;
  }
  // Prepare environmental parameters
  double tempC = bme.readTemperature();
  double pressurePa = bme.readPressure();
  double humidity = bme.readHumidity();
  double pressure_hPa = pressurePa / 100.0;
  double rho = airDensity(tempC, pressure_hPa, humidity);
  double sos = speedOfSound(tempC);
  // Use current bullet
  const Bullet& bullet = bulletList[currentBulletIndex];
  // Choose drag table based on bullet's drag model
  const DragEntry* dragTable;
  int dragCount;
  if (bullet.drag_model == 'G' && bullet.code[0] != '\0') {
    // bullet.drag_model is 'G', but we need to know G1 vs G7. We can check bc or code:
    // In our data, if bc is given, but let's decide based on bullet entry:
    // We stored 'G' for all, but original distinguishes by bullet.drag_model string "G1" or "G7".
    // We can derive from bullet struct: If bc is given but we can't know directly G1 vs G7 from a char.
    // Instead, we will rely on bullet code indices:
    // For bulletList we will manually set drag_model char to '1' or '7' for G1 vs G7 to differentiate.
  }
}

```cpp
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <LSM303.h>
#include <Preferences.h>

// OLED display setup for 0.91" SSD1306 (128x32)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BME280 environmental sensor (I2C address might be 0x76 or 0x77)
Adafruit_BME280 bme;
#define BME280_ADDR_PRIMARY   0x76
#define BME280_ADDR_SECONDARY 0x77

// LSM303DLHC accelerometer+magnetometer
LSM303 compass;
Preferences prefs;

// GPIO pins for buttons
#define PIN_BTN_UP      13
#define PIN_BTN_DOWN    12
#define PIN_BTN_LEFT    14
#define PIN_BTN_RIGHT   27
#define PIN_BTN_CENTER  26  // center (select)
#define PIN_BTN_BACK    25  // back
#define PIN_BTN_MODE    33  // mode toggle (manual/continuous, hold for calibrate)
#define PIN_BTN_TRIGGER 32  // external trigger for rangefinder

// UART for rangefinder (using Serial2 on ESP32)
HardwareSerial RangeSerial(2);
#define PIN_RANGE_TX 17  // TX pin (if needed to send commands)
#define PIN_RANGE_RX 16  // RX pin (to receive distance data)
#define RANGE_BAUD   9600  // baud rate for rangefinder (adjust if needed)

// Bullet profile structure
struct Bullet {
  const char* code;
  const char* name;
  double bc;
  double muzzle_velocity; // m/s
  double weight_gr;       // grains
  double sight_height;    // m
  double diameter;        // m
  char drag_model;        // '1' for G1, '7' for G7
};

// Bullet library (all bullets from original script)
static const Bullet bulletList[] = {
  { "308_M80",    ".308 M80 Ball",           0.400, 840.0, 147.0, 0.06985, 0.00782, '1' },
  { "308_SUB",    ".308 Subsonic",           0.518, 325.0, 200.0, 0.06985, 0.00782, '1' },
  { "300BLK_SUP", ".300 Blackout Supersonic",0.305, 670.0, 125.0, 0.06985, 0.00782, '7' },
  { "300BLK_SUB", ".300 Blackout Subsonic",  0.660, 305.0, 220.0, 0.06985, 0.00782, '1' },
  { "556_M193",   "5.56 NATO 55gr M193",     0.243, 990.0,  55.0, 0.06985, 0.00570, '1' },
  { "68BLK_SUP",  "6.8 Blackout Supersonic", 0.370, 780.0, 110.0, 0.06985, 0.00680, '7' },
  { "68BLK_SUB",  "6.8 Blackout Subsonic",   0.450, 330.0, 180.0, 0.06985, 0.00680, '1' }
};
const int NUM_BULLETS = sizeof(bulletList)/sizeof(Bullet);

// Drag tables (velocity in ft/s, Cd)
struct DragEntry { int vel_fps; double Cd; };
static const DragEntry G1_TABLE[] = {
  {137, 0.262}, {228, 0.230}, {274, 0.211}, {366, 0.203}, {457, 0.198},
  {549, 0.195}, {640, 0.192}, {732, 0.188}, {823, 0.185}, {914, 0.182},
  {1006, 0.179}, {1097, 0.176}, {1189, 0.172}, {1280, 0.169}, {1372, 0.165},
  {1463, 0.162}, {1555, 0.159}, {1646, 0.156}, {1738, 0.152}, {1829, 0.149},
  {1920, 0.147}, {2012, 0.144}, {2103, 0.141}
};
static const DragEntry G7_TABLE[] = {
  {137, 0.120}, {228, 0.112}, {274, 0.107}, {366, 0.104}, {457, 0.102},
  {549, 0.100}, {640, 0.098}, {732, 0.095}, {823, 0.093}, {914, 0.091},
  {1006,0.089}, {1097,0.087}, {1189,0.085}, {1280,0.083}, {1372,0.081},
  {1463,0.079}, {1555,0.077}, {1646,0.075}, {1738,0.073}, {1829,0.071},
  {1920,0.069}, {2012,0.067}, {2103,0.065}
};
const int G1_COUNT = sizeof(G1_TABLE)/sizeof(DragEntry);
const int G7_COUNT = sizeof(G7_TABLE)/sizeof(DragEntry);

// Global state
int currentBulletIndex = 0;
bool continuousMode = false;
double windSpeed = 0.0;    // wind speed (m/s)
double windDirDeg = 0.0;   // wind direction (degrees, coming from)
double lastRange = 0.0;    // last measured range (m)
bool haveRange = false;
char lastSolutionStr[16] = "";
bool editing = false;
int selectedField = 0;  // 1=Bullet, 2=Wind Speed, 3=Wind Dir

// Button state tracking (for edge detection and long press)
bool lastStateUp = false, lastStateDown = false;
bool lastStateLeft = false, lastStateRight = false;
bool lastStateCenter = false, lastStateBack = false;
bool lastStateMode = false, lastStateTrigger = false;
unsigned long modePressStart = 0;
bool modeHoldHandled = false;

// Magnetometer calibration
LSM303::vector<int16_t> magMin, magMax;
bool magCalibrated = false;

// Environmental constants
const double R_AIR = 287.05;  // J/(kg·K), specific gas constant for dry air

// Utility: speed of sound (approx) at given temperature (°C)
double speedOfSound(double tempC) {
  return sqrt(1.4 * R_AIR * (tempC + 273.15));
}

// Utility: air density (kg/m^3) given temp (°C), pressure (hPa), humidity (%)
double airDensity(double tempC, double pressure_hPa, double humidity) {
  double T = tempC + 273.15;
  double P = pressure_hPa * 100.0;  // convert hPa to Pa
  // Saturation vapor pressure (hPa) at tempC (Tetens formula)
  double saturation_hPa = 6.1078 * pow(10.0, (7.5 * tempC) / (tempC + 237.3));
  double vaporPressure = (humidity / 100.0) * saturation_hPa * 100.0;  // Pa
  double dryAirPressure = P - vaporPressure;
  return dryAirPressure / (R_AIR * T);
}

// Utility: interpolate drag coefficient for given velocity (m/s) from drag table
double interpolateCd(double velocity_mps, const DragEntry* table, int tableSize) {
  double v_fps = velocity_mps * 3.28084;
  for (int i = 0; i < tableSize - 1; ++i) {
    if (v_fps >= table[i].vel_fps && v_fps <= table[i+1].vel_fps) {
      double v1 = table[i].vel_fps;
      double v2 = table[i+1].vel_fps;
      double cd1 = table[i].Cd;
      double cd2 = table[i+1].Cd;
      // linear interpolation
      return cd1 + (cd2 - cd1) * ((v_fps - v1) / (v2 - v1));
    }
  }
  // If velocity beyond last entry, return last Cd
  return table[tableSize - 1].Cd;
}

// Utility: apply transonic drag correction
double transonicCorrection(double cd, double mach) {
  if (mach >= 0.8 && mach <= 1.2) {
    return cd * 1.15;  // increase drag by 15% in transonic range
  }
  return cd;
}

// Compute ballistic drop and drift in mils for current state
void computeBallistic(double range_m, double pitchRad, double headingDeg,
                      double windSpeed_mps, double windDirDeg,
                      double& outDropMils, double& outDriftMils) {
  if (range_m <= 0) {
    outDropMils = 0.0;
    outDriftMils = 0.0;
    return;
  }
  // Determine horizontal distance and target elevation relative to shooter
  double horizontalRange = range_m * cos(pitchRad);
  double targetElevation = range_m * sin(pitchRad);
  // Use current bullet parameters
  const Bullet& bullet = bulletList[currentBulletIndex];
  // Environmental measurements
  double tempC = bme.readTemperature();
  double pressure_hPa = bme.readPressure() / 100.0;
  double humidity = bme.readHumidity();
  double rho = airDensity(tempC, pressure_hPa, humidity);
  double sos = speedOfSound(tempC);
  // Bullet parameters for simulation
  double g = 9.81;
  double dt = 0.01;
  double x = 0.0;
  double y = bullet.sight_height;
  double v = bullet.muzzle_velocity;
  // Convert bullet weight to kg
  double mass = bullet.weight_gr * 0.00006479891;
  // Cross-sectional area of bullet (m^2)
  double A = PI * pow(bullet.diameter / 2.0, 2);
  // Wind relative angle: compute angle between wind direction and shooter's heading
  // relativeWindDeg = headingDeg - windDirDeg (mod 360)
  double relativeWindDeg = fmod((headingDeg - windDirDeg + 360.0), 360.0);
  double windAngleRad = relativeWindDeg * DEG_TO_RAD;
  double drift = 0.0;
  // Select drag table based on G1 or G7
  const DragEntry* dragTable = (bullet.drag_model == '1') ? G1_TABLE : G7_TABLE;
  int dragSize = (bullet.drag_model == '1') ? G1_COUNT : G7_COUNT;
  // Integrate trajectory until horizontal distance reached or velocity drops too low
  while (x < horizontalRange && v > 100.0) {
    double mach = v / sos;
    double Cd = interpolateCd(v, dragTable, dragSize);
    Cd = transonicCorrection(Cd, mach);
    // Drag force Fd = 0.5 * rho * Cd * A * v^2
    double Fd = 0.5 * rho * Cd * A * v * v;
    double a_drag = Fd / mass;
    // Update velocity and positions
    v -= a_drag * dt;
    y -= g * dt;
    drift += windSpeed_mps * sin(windAngleRad) * dt;
    x += v * dt;
    // (time t advanced by dt if needed, but we don't use time output)
  }
  // Compute drop and drift
  double drop = bullet.sight_height - y - targetElevation;
  double mils = (horizontalRange > 0.0) ? (drop / horizontalRange) * 1000.0 : 0.0;
  double drift_mils = (horizontalRange > 0.0) ? (drift / horizontalRange) * 1000.0 : 0.0;
  outDropMils = mils;
  outDriftMils = drift_mils;
}

// Draw the current UI (ballistic solution and fields)
void drawDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Prepare strings for fields
  const Bullet& bullet = bulletList[currentBulletIndex];
  char bulletStr[20];
  snprintf(bulletStr, sizeof(bulletStr), "%s", bullet.code);  // bullet code or short name
  char rangeStr[10];
  if (haveRange) {
    // display range in meters with no decimal
    snprintf(rangeStr, sizeof(rangeStr), "%.0fm", lastRange);
  } else {
    snprintf(rangeStr, sizeof(rangeStr), "---m");
  }
  char windStr[20];
  snprintf(windStr, sizeof(windStr), "%.0fm/s %.0fdeg", windSpeed, windDirDeg);
  // We also have the solution string ready (lastSolutionStr updated after computeBallistic)

  // We will display:
  // Line1: Bullet code and range
  // Line2: Wind info
  // Line3: Solution (elevation and windage)
  // Line4: Mode indicator or empty (and used for highlighting if needed)
  // If editing, we highlight fields accordingly by inverting their text.

  // Line 1: Bullet and Range
  display.setCursor(0, 0);
  if (editing && selectedField == 1) {
    // Highlight bullet (invert bullet text)
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.print(bulletStr);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.print(" ");
    display.print(rangeStr);
    display.setTextColor(SSD1306_WHITE);
  } else {
    // Not editing bullet, or not selected
    display.print(bulletStr);
    display.print(" ");
    display.print(rangeStr);
  }

  // Line 2: Wind
  display.setCursor(0, 8);
  display.print("Wind: ");
  // Determine which part of wind is highlighted if editing
  if (editing && (selectedField == 2 || selectedField == 3)) {
    // We will split wind into two parts: speed and direction
    // Find the position of space between speed and direction in windStr
    // windStr format: "<speed>m/s <dir>deg"
    char* dirPart = strstr(windStr, " ");
    if (dirPart) {
      *dirPart = '\0'; // temporarily split
      const char* speedPart = windStr;
      const char* directionPart = dirPart + 1;
      if (selectedField == 2) {
        // Highlight wind speed
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(speedPart);
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.print(" ");
        display.print(directionPart);
        display.setTextColor(SSD1306_WHITE);
      } else if (selectedField == 3) {
        // Highlight wind direction
        display.print(speedPart);
        display.print(" ");
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(directionPart);
        display.setTextColor(SSD1306_WHITE);
      }
      *dirPart = ' '; // restore string
    } else {
      // safety: if format unexpected, just print normally
      display.print(windStr);
    }
  } else {
    // Not editing wind fields
    display.print(windStr);
  }

  // Line 3: Solution
  display.setCursor(0, 16);
  // Determine vertical and horizontal adjustments and format them
  // lastSolutionStr is maintained globally after last computeBallistic
  display.print(lastSolutionStr);

  // Line 4: Mode indicator (manual/continuous) if not editing (or calibrating)
  display.setCursor(0, 24);
  if (continuousMode) {
    display.print("Mode: Continuous");
  } else {
    display.print("Mode: Manual");
  }

  display.display();
}

// Read rangefinder serial data (non-blocking). Returns true if a new range was read.
bool readRangefinder() {
  // Check if any data available on rangefinder serial
  static String incoming = "";
  bool gotReading = false;
  while (RangeSerial.available()) {
    char c = (char)RangeSerial.read();
    if (c == '\r' || c == '\n') {
      if (incoming.length() > 0) {
        // parse number from incoming string
        double distance = incoming.toDouble();
        if (distance > 0) {
          lastRange = distance;
          haveRange = true;
          gotReading = true;
        }
      }
      incoming = "";
    } else {
      incoming += c;
      // safety: limit length to avoid overflow for unexpected data
      if (incoming.length() > 20) incoming.remove(0, incoming.length());
    }
  }
  return gotReading;
}

// Handle entering calibration mode
void startCalibration() {
  calibrating = true;
  // Reset min/max
  magMin = {+32767, +32767, +32767};
  magMax = {-32768, -32768, -32768};
  // Display instructions
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8);
  display.println("Calibrating...");
  display.setCursor(0, 16);
  display.println("Rotate sensor");
  display.display();
}

// Handle the calibration loop (should be called frequently while calibrating)
void performCalibration() {
  // Read magnetometer (and accelerometer, though we only need magnetometer for calibration)
  compass.read();
  // Update min/max for magnetometer
  if (compass.m.x < magMin.x) magMin.x = compass.m.x;
  if (compass.m.y < magMin.y) magMin.y = compass.m.y;
  if (compass.m.z < magMin.z) magMin.z = compass.m.z;
  if (compass.m.x > magMax.x) magMax.x = compass.m.x;
  if (compass.m.y > magMax.y) magMax.y = compass.m.y;
  if (compass.m.z > magMax.z) magMax.z = compass.m.z;
}

// Finish calibration: save values and apply
void finishCalibration() {
  calibrating = false;
  // Save calibration to preferences
  prefs.putShort("magMinX", magMin.x);
  prefs.putShort("magMinY", magMin.y);
  prefs.putShort("magMinZ", magMin.z);
  prefs.putShort("magMaxX", magMax.x);
  prefs.putShort("magMaxY", magMax.y);
  prefs.putShort("magMaxZ", magMax.z);
  prefs.putBool("magCal", true);
  // Apply calibration to compass object
  compass.m_min = magMin;
  compass.m_max = magMax;
  magCalibrated = true;
  // Confirm done on display briefly
  display.clearDisplay();
  display.setCursor(0, 8);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Calibration done");
  display.display();
  delay(1000);
  // Redraw normal display after calibration
  drawDisplay();
}

// Setup hardware
void setup() {
  // Initialize serial for debugging (optional)
  // Serial.begin(115200);
  // Serial.println("Starting Ballistica ESP32...");

  // Initialize display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // if display not found, hang
    // Serial.println("SSD1306 not found");
    while(true);
  }
  display.clearDisplay();
  display.display();

  // Initialize I2C sensors
  Wire.begin();
  bool bmeOK = bme.begin(BME280_ADDR_PRIMARY);
  if (!bmeOK) {
    bmeOK = bme.begin(BME280_ADDR_SECONDARY);
  }
  // It's okay if BME280 fails (should not, if connected properly)
  compass.init();
  compass.enableDefault();

  // Initialize rangefinder serial
  RangeSerial.begin(RANGE_BAUD, SERIAL_8N1, PIN_RANGE_RX, PIN_RANGE_TX);

  // Setup button inputs (use internal pull-ups)
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_CENTER, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK, INPUT_PULLUP);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_TRIGGER, INPUT_PULLUP);

  // Load saved preferences
  prefs.begin("Ballistica", false);
  currentBulletIndex = prefs.getInt("bulletIndex", 0);
  if (currentBulletIndex < 0 || currentBulletIndex >= NUM_BULLETS) currentBulletIndex = 0;
  windSpeed = prefs.getDouble("windSpeed", 0.0);
  windDirDeg = prefs.getDouble("windDir", 0.0);
  // Load magnetometer calibration if exists
  if (prefs.getBool("magCal", false)) {
    magMin.x = prefs.getShort("magMinX", 0);
    magMin.y = prefs.getShort("magMinY", 0);
    magMin.z = prefs.getShort("magMinZ", 0);
    magMax.x = prefs.getShort("magMaxX", 0);
    magMax.y = prefs.getShort("magMaxY", 0);
    magMax.z = prefs.getShort("magMaxZ", 0);
    compass.m_min = magMin;
    compass.m_max = magMax;
    magCalibrated = true;
  }

  // Initial draw of UI
  strcpy(lastSolutionStr, "----");  // no solution yet
  drawDisplay();
}

// Poll buttons and handle UI logic
void loop() {
  // If in calibration mode, handle calibration process
  if (calibrating) {
    // Check for back button to finish calibration
    bool backPressed = (digitalRead(PIN_BTN_BACK) == LOW);
    if (backPressed) {
      finishCalibration();
      // Consume this back press so it doesn't act in normal mode
      lastStateBack = backPressed;
      return;
    }
    // Continue calibration updates at a limited rate
    static unsigned long lastCalUpdate = 0;
    if (millis() - lastCalUpdate >= 100) {
      performCalibration();
      lastCalUpdate = millis();
    }
    // In calibration mode, do not update main display or ballistic calc
    return;
  }

  // Read current button states (active LOW)
  bool stateUp      = (digitalRead(PIN_BTN_UP) == LOW);
  bool stateDown    = (digitalRead(PIN_BTN_DOWN) == LOW);
  bool stateLeft    = (digitalRead(PIN_BTN_LEFT) == LOW);
  bool stateRight   = (digitalRead(PIN_BTN_RIGHT) == LOW);
  bool stateCenter  = (digitalRead(PIN_BTN_CENTER) == LOW);
  bool stateBack    = (digitalRead(PIN_BTN_BACK) == LOW);
  bool stateMode    = (digitalRead(PIN_BTN_MODE) == LOW);
  bool stateTrigger = (digitalRead(PIN_BTN_TRIGGER) == LOW);

  // Handle Mode button (toggle manual/continuous or long-press for calibration)
  if (stateMode && !lastStateMode) {
    // Mode button just pressed
    modePressStart = millis();
    modeHoldHandled = false;
  }
  if (stateMode && !modeHoldHandled && millis() - modePressStart > 3000) {
    // Long press detected - enter calibration
    modeHoldHandled = true;
    startCalibration();
    // Do not toggle continuous mode on release in this case
  }
  if (!stateMode && lastStateMode) {
    // Mode button released
    if (!modeHoldHandled) {
      // Short press (no calibrate triggered) - toggle mode
      continuousMode = !continuousMode;
      // Indicate mode change (e.g., we could flash or just update display)
      // Mark display for update
      drawDisplay();
    }
    modePressStart = 0;
    modeHoldHandled = false;
  }

  // Handle Trigger button (trigger rangefinder reading)
  bool newRange = false;
  if (stateTrigger && !lastStateTrigger) {
    // If trigger pressed, we could send a command to rangefinder if needed (depending on device).
    // If the rangefinder requires an external trigger signal and is connected separately, that would be hardware-specific.
    // Here we assume either the rangefinder automatically sends data when triggered by user, or we simply read available data.
  }
  // Continuously read from rangefinder serial if data available
  newRange = readRangefinder();
  if (newRange) {
    // If a new range was obtained, recalc solution if manual mode
    if (!continuousMode) {
      // Recalculate ballistic solution for new range immediately
      // Get current orientation (pitch from accelerometer, heading from magnetometer)
      compass.read();
      // Apply calibration if available for heading calculation
      if (magCalibrated) {
        compass.m_min = magMin;
        compass.m_max = magMax;
      }
      float pitchRad = 0.0;
      // Calculate pitch from accelerometer (assuming X axis is forward, Z axis up)
      // pitch = angle above horizontal. Using formula: pitch = atan2(Xacc, Zacc)
      float ax = compass.a.x;
      float az = compass.a.z;
      // If accelerometer Z is pointing upward, gravity produces a negative acceleration in Z when level.
      // We might invert sign for calculation if needed. Let's assume calibration yields positive az for upward acceleration of device.
      pitchRad = atan2(-ax, sqrt((double)compass.a.y * compass.a.y + (double)compass.a.z * compass.a.z));
      // Actually, use formula pitch = atan2(-Ax, sqrt(Ay^2+Az^2)) to get pitch angle in radians
      // (This formula expects coordinate system X: forward, Y: right, Z: down. If our Z is up, Az might be negative of down)
      // We'll apply it directly:
      pitchRad = atan2(- (double)compass.a.x, sqrt((double)compass.a.y * compass.a.y + (double)compass.a.z * compass.a.z));
      // Compute heading (0-360)
      float headingDeg;
      // Use compass.heading() function from LSM303 library for tilt-compensated heading
      // The Pololu LSM303 library uses calibrated values (compass.m_min/m_max) for heading.
      headingDeg = compass.heading((LSM303::vector<int>){0, 0, 1});
      // Calculate ballistic solution
      double dropMils, driftMils;
      computeBallistic(lastRange, pitchRad, headingDeg, windSpeed, windDirDeg, dropMils, driftMils);
      // Format solution string (e.g., "3.4U -5.8L" or "3.4U 5.8R")
      char vertDir = (dropMils >= 0.0) ? 'U' : 'D';
      char horizDir = (driftMils >= 0.0) ? 'L' : 'R';
      double vertAdj = fabs(dropMils);
      double horizAdj = fabs(driftMils);
      snprintf(lastSolutionStr, sizeof(lastSolutionStr), "%.1f%c %.1f%c", vertAdj, vertDir, horizAdj, horizDir);
      // Update display
      drawDisplay();
    }
  }

  // Handle arrow buttons for navigation/adjustment
  if (stateCenter && !lastStateCenter) {
    // Center pressed: toggle editing mode or confirm selection
    if (!editing) {
      // Enter editing mode, select first field (Bullet)
      editing = true;
      selectedField = 1;
    } else {
      // Already editing - maybe use center to toggle through fields?
      // We can choose to advance to next field on each press, or exit edit mode.
      // Here, we will cycle fields on center press.
      selectedField++;
      if (selectedField > 3) {
        // If beyond last field, exit editing
        editing = false;
        selectedField = 0;
        // Save changed values
        prefs.putInt("bulletIndex", currentBulletIndex);
        prefs.putDouble("windSpeed", windSpeed);
        prefs.putDouble("windDir", windDirDeg);
      }
    }
    drawDisplay();
  }

  if (stateBack && !lastStateBack) {
    // Back pressed: if editing, cancel/exit editing without saving (or simply exit)
    if (editing) {
      editing = false;
      selectedField = 0;
      // We might revert changes if we had a mechanism, but since we apply immediately on adjust, just exit.
      drawDisplay();
    }
    // If not editing, back could be used to do nothing (or reserved for calibrate exit which we handle separately).
  }

  if (editing) {
    // Navigation within editing mode
    if (stateUp && !lastStateUp) {
      // Move selection up (previous field)
      selectedField--;
      if (selectedField < 1) selectedField = 3;
      drawDisplay();
    }
    if (stateDown && !lastStateDown) {
      // Move selection down (next field)
      selectedField++;
      if (selectedField > 3) selectedField = 1;
      drawDisplay();
    }
    if ((stateLeft && !lastStateLeft) || (stateRight && !lastStateRight)) {
      // Adjust value of selected field
      int direction = 0;
      if (stateLeft && !lastStateLeft) direction = -1;
      if (stateRight && !lastStateRight) direction = 1;
      if (selectedField == 1) {
        // Change bullet selection
        currentBulletIndex += direction;
        if (currentBulletIndex < 0) currentBulletIndex = NUM_BULLETS - 1;
        if (currentBulletIndex >= NUM_BULLETS) currentBulletIndex = 0;
      } else if (selectedField == 2) {
        // Adjust wind speed (m/s)
        windSpeed += direction * 1.0; // change by 1 m/s step
        if (windSpeed < 0) windSpeed = 0;
      } else if (selectedField == 3) {
        // Adjust wind direction (degrees)
        windDirDeg += direction * 5.0; // change by 5 degrees step
        if (windDirDeg < 0) windDirDeg += 360;
        if (windDirDeg >= 360) windDirDeg -= 360;
      }
      // Immediately recalc ballistic solution if in manual mode (since continuous mode will handle itself)
      if (!continuousMode && haveRange) {
        compass.read();
        if (magCalibrated) {
          compass.m_min = magMin;
          compass.m_max = magMax;
        }
        // Recompute pitch & heading
        double pitchRad = atan2(-(double)compass.a.x, sqrt((double)compass.a.y * compass.a.y + (double)compass.a.z * compass.a.z));
        double headingDeg = compass.heading((LSM303::vector<int>){0, 0, 1});
        double dropMils, driftMils;
        computeBallistic(lastRange, pitchRad, headingDeg, windSpeed, windDirDeg, dropMils, driftMils);
        char vertDir = (dropMils >= 0.0) ? 'U' : 'D';
        char horizDir = (driftMils >= 0.0) ? 'L' : 'R';
        double vertAdj = fabs(dropMils);
        double horizAdj = fabs(driftMils);
        snprintf(lastSolutionStr, sizeof(lastSolutionStr), "%.1f%c %.1f%c", vertAdj, vertDir, horizAdj, horizDir);
      }
      // Update display to reflect changed value and possibly new solution
      drawDisplay();
    }
  }

  // Continuous mode: update ballistic solution continuously
  static unsigned long lastContUpdate = 0;
  if (continuousMode && (millis() - lastContUpdate >= 100)) {
    lastContUpdate = millis();
    // Read orientation and environment continuously
    compass.read();
    if (magCalibrated) {
      compass.m_min = magMin;
      compass.m_max = magMax;
    }
    // Compute pitch and heading
    double pitchRad = atan2(-(double)compass.a.x, sqrt((double)compass.a.y * compass.a.y + (double)compass.a.z * compass.a.z));
    double headingDeg = compass.heading((LSM303::vector<int>){0, 0, 1});
    // If in continuous mode, we might not have an automatically changing range unless rangefinder provides streaming.
    // We will use lastRange (from last trigger press or measurement).
    if (!haveRange) {
      // If no range measured yet, skip until we have one
    } else {
      double dropMils, driftMils;
      computeBallistic(lastRange, pitchRad, headingDeg, windSpeed, windDirDeg, dropMils, driftMils);
      char vertDir = (dropMils >= 0.0) ? 'U' : 'D';
      char horizDir = (driftMils >= 0.0) ? 'L' : 'R';
      double vertAdj = fabs(dropMils);
      double horizAdj = fabs(driftMils);
      char solutionBuf[16];
      snprintf(solutionBuf, sizeof(solutionBuf), "%.1f%c %.1f%c", vertAdj, vertDir, horizAdj, horizDir);
      // Only update display if the solution or other displayed info has changed (to avoid flicker)
      if (strcmp(solutionBuf, lastSolutionStr) != 0 ||
          editing || !continuousMode) {
        strcpy(lastSolutionStr, solutionBuf);
        drawDisplay();
      }
    }
  }

  // Save last states for next loop
  lastStateUp = stateUp;
  lastStateDown = stateDown;
  lastStateLeft = stateLeft;
  lastStateRight = stateRight;
  lastStateCenter = stateCenter;
  lastStateBack = stateBack;
  lastStateMode = stateMode;
  lastStateTrigger = stateTrigger;
}
