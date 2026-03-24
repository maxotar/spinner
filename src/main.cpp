#include <Arduino.h>
#include <TMCStepper.h>
#include <Adafruit_seesaw.h>
#include <Adafruit_LEDBackpack.h>

// ================================================================
// Feather M4 + TMC2209 + Joy FeatherWing + Alphanumeric Display
//
// Uses TMC2209 VACTUAL velocity mode — the chip drives the motor
// from its own internal oscillator. No STEP pulses or hardware
// timer needed. DIR and STEP pins are ignored in this mode;
// direction is set via driver.shaft().
//
// Controls:
//   UP button    — increase speed
//   DOWN button  — decrease speed (reaches 0 = stopped)
//   SELECT       — immediately stop
//
// Speed is stored as centi-RPM (units of 0.01 RPM) to avoid
// float drift from repeated addition.
//
// VACTUAL conversion:
//   VACTUAL = crpm * FULL_STEPS * MICROSTEPS * 2^24
//             ------------------------------------------
//             fclk * 60 * 100
//
//   With fclk=12MHz (TMC2209 internal oscillator, typical),
//   200 full steps, 8 microsteps:
//   VACTUAL_PER_CRPM ≈ 0.3728
//
//   Minimum achievable speed ≈ 0.03 RPM (VACTUAL must be >= 1).
//   Increase MICROSTEPS for finer low-speed resolution.
//
// Step size bands:
//   0.03 – 0.09 RPM  →  0.01 RPM per press
//   0.10 – 0.99 RPM  →  0.10 RPM per press
//   1.0  – 49.9 RPM  →  1.0  RPM per press
//   50+  RPM         →  10.0 RPM per press
//   Max speed: 1200 RPM
// ================================================================

// --- EN pin (STEP and DIR unused in VACTUAL mode) ---
#define EN_PIN 9

// --- Joy FeatherWing seesaw pins ---
// Confirmed pinout for this hardware revision:
//   Joystick UD = pin 2 (low=up, high=down)
//   Joystick LR = pin 3 (low=left, high=right)
//   Buttons are active LOW with internal pullups enabled
#define SEESAW_ADDR 0x49
#define BUTTON_RIGHT 6
#define BUTTON_DOWN 7
#define BUTTON_LEFT 9
#define BUTTON_UP 10
#define BUTTON_SEL 14 // Dedicated SELECT button

#define BUTTON_MASK ((1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) | \
                     (1 << BUTTON_LEFT) | (1 << BUTTON_UP) |    \
                     (1 << BUTTON_SEL))

Adafruit_seesaw ss;

// --- Alphanumeric display (HT16K33) ---
#define DISPLAY_ADDR 0x70
#define BRIGHTNESS 1 // 0-15; 1 = ~6%, lowest non-off setting
Adafruit_AlphaNum4 display;

// --- TMC2209 ---
#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f
#define RMS_CURRENT 1300 // mA — raised to help with high-speed torque
                         // Max rated RMS for 17HS19-2004S1 is 1414mA
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

// --- Motor constants ---
const int FULL_STEPS_PER_REV = 200; // 1.8 degree motor
const int MICROSTEPS = 8;           // Higher = smoother/quieter but higher min speed

// --- Drive mode ---
// Below threshold: StealthChop (quiet, less torque at speed)
// Above threshold: SpreadCycle (louder, full torque)
// Set to 0 to always use SpreadCycle.
// Set to CRPM_MAX to always use StealthChop.
const int SPREADCYCLE_THRESHOLD_CRPM = 20000; // 200.00 RPM
bool spreadCycleActive = false;

// Set to true to reverse motor direction
const bool REVERSE_DIRECTION = false;

// --- VACTUAL conversion ---
// TMC2209 internal oscillator: 12 MHz nominal (+-15% typical, untrimmed).
// Adjust FCLK_HZ if you calibrate the actual clock and need better accuracy.
const float FCLK_HZ = 12000000.0f;
const float VACTUAL_PER_CRPM = (FULL_STEPS_PER_REV * MICROSTEPS * 16777216.0f) / (FCLK_HZ * 60.0f * 100.0f);

// --- Speed state (centi-RPM = units of 0.01 RPM) ---
int speedCRpm = 0;
int lastDisplayedCRpm = -1;
// CRPM_MIN must produce VACTUAL >= 1.
// At 8 microsteps / 12MHz the minimum is ~0.03 RPM (crpm=3).
const int CRPM_MIN = 3;
const int CRPM_MAX = 120000; // 1200.00 RPM

// ----------------------------------------------------------------
// Motor speed helpers
// ----------------------------------------------------------------

int32_t crpmToVactual(int crpm)
{
  return (int32_t)(crpm * VACTUAL_PER_CRPM);
}

void setMotorSpeed(int crpm)
{
  if (crpm == 0)
  {
    driver.VACTUAL(0);
  }
  else
  {
    int32_t vactual = crpmToVactual(crpm);
    vactual = constrain(vactual, 1, 8388607); // 23-bit signed max
    driver.VACTUAL(vactual);
  }
}

// Freewheeling: EN_PIN HIGH cuts all coil current so the motor
// spins freely. Re-enabled automatically when speed is next changed.
bool freewheeling = false;

void setFreewheel(bool enable)
{
  freewheeling = enable;
  digitalWrite(EN_PIN, enable ? HIGH : LOW);
  Serial.println(enable ? "Freewheeling." : "Driver re-enabled.");
}

void updateDriveMode(int crpm)
{
  bool needsSpreadCycle = (crpm >= SPREADCYCLE_THRESHOLD_CRPM);
  if (needsSpreadCycle != spreadCycleActive)
  {
    spreadCycleActive = needsSpreadCycle;
    driver.en_spreadCycle(spreadCycleActive);
    Serial.println(spreadCycleActive ? "-> SpreadCycle" : "-> StealthChop");
  }
}

// Returns increment size in centi-RPM for the current speed level
int getStepCRpm(int crpm)
{
  if (crpm < 10)
    return 1; // 0.01 RPM steps
  if (crpm < 100)
    return 10; // 0.10 RPM steps
  if (crpm < 5000)
    return 100; // 1.0  RPM steps
  return 1000;  // 10.0 RPM steps
}

void applyChange(int dir)
{
  if (freewheeling)
    setFreewheel(false); // Re-enable driver on any speed input
  if (dir > 0)
  {
    speedCRpm = (speedCRpm == 0) ? CRPM_MIN
                                 : min(speedCRpm + getStepCRpm(speedCRpm), CRPM_MAX);
  }
  else
  {
    if (speedCRpm == 0)
      return;
    speedCRpm = max(speedCRpm - getStepCRpm(speedCRpm), 0);
  }
  updateDriveMode(speedCRpm);
  setMotorSpeed(speedCRpm);
  Serial.print("RPM: ");
  Serial.println(speedCRpm / 100.0f, 2);
}

// ----------------------------------------------------------------
// Display
// Formats:
//   freewheeling -> "FREE"
//   stopped      -> "----"
//   < 1.0        -> "0.XX"   e.g. 0.03, 0.75
//   < 10.0       -> " X.X"   e.g.  1.5
//   < 100.0      -> "XX.X"   e.g. 10.5
//   >= 100       -> " XXX"   e.g.  100, 999
//   >= 1000      -> "XXXX"   e.g. 1200
// ----------------------------------------------------------------
void updateDisplay(int crpm)
{
  if (crpm <= 0)
  {
    display.writeDigitAscii(0, '-');
    display.writeDigitAscii(1, '-');
    display.writeDigitAscii(2, '-');
    display.writeDigitAscii(3, '-');
  }
  else if (crpm < 100)
  {
    // "0.XX"
    int v = constrain(crpm, 1, 99);
    display.writeDigitAscii(0, '0', true);
    display.writeDigitAscii(1, '0' + v / 10);
    display.writeDigitAscii(2, '0' + v % 10);
    display.writeDigitAscii(3, ' ');
  }
  else if (crpm < 1000)
  {
    // " X.X"
    int v = constrain((crpm + 5) / 10, 10, 99);
    display.writeDigitAscii(0, ' ');
    display.writeDigitAscii(1, '0' + v / 10, true);
    display.writeDigitAscii(2, '0' + v % 10);
    display.writeDigitAscii(3, ' ');
  }
  else if (crpm < 10000)
  {
    // "XX.X"
    int v = constrain((crpm + 5) / 10, 100, 999);
    display.writeDigitAscii(0, '0' + v / 100);
    display.writeDigitAscii(1, '0' + (v / 10) % 10, true);
    display.writeDigitAscii(2, '0' + v % 10);
    display.writeDigitAscii(3, ' ');
  }
  else if (crpm < 100000)
  {
    // " XXX"  (100 - 999 RPM)
    int v = constrain((crpm + 50) / 100, 100, 999);
    display.writeDigitAscii(0, ' ');
    display.writeDigitAscii(1, '0' + v / 100);
    display.writeDigitAscii(2, '0' + (v / 10) % 10);
    display.writeDigitAscii(3, '0' + v % 10);
  }
  else
  {
    // "XXXX"  (1000+ RPM)
    int v = constrain((crpm + 50) / 100, 1000, 1200);
    display.writeDigitAscii(0, '0' + v / 1000);
    display.writeDigitAscii(1, '0' + (v / 100) % 10);
    display.writeDigitAscii(2, '0' + (v / 10) % 10);
    display.writeDigitAscii(3, '0' + v % 10);
  }

  display.writeDisplay();
}

// ----------------------------------------------------------------
// Button repeat
// A held button fires once immediately, pauses, then repeats.
// ----------------------------------------------------------------
const unsigned long REPEAT_DELAY_MS = 400; // Pause before repeat starts
const unsigned long REPEAT_RATE_MS = 80;   // Interval once repeating

struct ButtonRepeat
{
  bool held;
  unsigned long pressTime;
  unsigned long lastRepeat;
  bool repeatStarted;
};
ButtonRepeat upState = {false, 0, 0, false};
ButtonRepeat downState = {false, 0, 0, false};

void handleRepeat(ButtonRepeat &state, uint32_t buttons, uint8_t pin, int dir)
{
  bool pressed = !(buttons & (1 << pin));

  if (pressed && !state.held)
  {
    // Fresh press — fire immediately
    state.held = true;
    state.repeatStarted = false;
    state.pressTime = millis();
    state.lastRepeat = millis();
    applyChange(dir);
  }
  else if (pressed && state.held)
  {
    // Held — wait for initial delay then repeat
    if (!state.repeatStarted && (millis() - state.pressTime) >= REPEAT_DELAY_MS)
      state.repeatStarted = true;
    if (state.repeatStarted && (millis() - state.lastRepeat) >= REPEAT_RATE_MS)
    {
      applyChange(dir);
      state.lastRepeat = millis();
    }
  }
  else
  {
    // Released
    state.held = false;
    state.repeatStarted = false;
  }
}

// --- Seesaw poll throttle ---
const unsigned long SEESAW_INTERVAL_MS = 20; // 50Hz
unsigned long lastSeesawRead = 0;
uint32_t prevButtons = BUTTON_MASK; // All HIGH = none pressed

// ----------------------------------------------------------------
// Setup
// ----------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;

  // Joy FeatherWing
  if (!ss.begin(SEESAW_ADDR))
  {
    Serial.println("Joy FeatherWing not found!");
    while (1)
      delay(10);
  }
  ss.pinModeBulk(BUTTON_MASK, INPUT_PULLUP);
  Serial.println("Joy FeatherWing OK.");

  // Alphanumeric display
  if (!display.begin(DISPLAY_ADDR))
  {
    Serial.println("Display not found!");
    while (1)
      delay(10);
  }
  display.setBrightness(BRIGHTNESS);
  updateDisplay(0);
  Serial.println("Display OK.");

  // TMC2209
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Active LOW — enable driver

  Serial1.begin(115200);
  driver.begin();
  driver.rms_current(RMS_CURRENT);
  driver.microsteps(MICROSTEPS);
  driver.en_spreadCycle(false); // Start in StealthChop
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);       // Tunes PWM to motor over time, reduces noise
  driver.shaft(REVERSE_DIRECTION); // Direction in VACTUAL mode
  driver.VACTUAL(0);               // Start stopped

  uint32_t ver = driver.version();
  Serial.print("Driver version: 0x");
  Serial.println(ver, HEX);
  if (ver != 0x21)
    Serial.println("WARNING: UART comms failed -- check wiring");

  Serial.print("SpreadCycle threshold: ");
  Serial.print(SPREADCYCLE_THRESHOLD_CRPM / 100.0f, 1);
  Serial.println(" RPM");
  Serial.print("VACTUAL_PER_CRPM: ");
  Serial.println(VACTUAL_PER_CRPM, 4);
  Serial.println("Ready.  UP=faster  DOWN=slower  SELECT=freewheel");
}

// ----------------------------------------------------------------
// Loop
// ----------------------------------------------------------------
void loop()
{

  // Display update runs freely — no need to throttle
  if (speedCRpm != lastDisplayedCRpm || freewheeling)
  {
    if (freewheeling)
    {
      display.writeDigitAscii(0, 'F');
      display.writeDigitAscii(1, 'R');
      display.writeDigitAscii(2, 'E');
      display.writeDigitAscii(3, 'E');
      display.writeDisplay();
    }
    else
    {
      updateDisplay(speedCRpm);
    }
    lastDisplayedCRpm = speedCRpm;
  }

  // Throttle all seesaw I2C reads to 50Hz
  if (millis() - lastSeesawRead < SEESAW_INTERVAL_MS)
    return;
  lastSeesawRead = millis();

  uint32_t buttons = ss.digitalReadBulk(BUTTON_MASK);
  uint32_t justPressed = prevButtons & ~buttons; // Bits that went LOW this cycle
  prevButtons = buttons;

  // SELECT: stop and freewheel (cuts coil current, motor spins freely)
  if (justPressed & (1 << BUTTON_SEL))
  {
    speedCRpm = 0;
    updateDriveMode(0);
    setMotorSpeed(0);
    setFreewheel(true);
  }

  // UP / DOWN with hold-to-repeat
  handleRepeat(upState, buttons, BUTTON_UP, +1);
  handleRepeat(downState, buttons, BUTTON_DOWN, -1);
}