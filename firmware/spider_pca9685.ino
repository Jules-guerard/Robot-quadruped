/*
  Spider V4 -> Arduino Nano (ATmega328P) + PCA9685 (I2C)

  Original project: SunFounder Crawling Robot
  Original author: panerqiang@sunfounder.com (2015)
  Spider project modifications: Regis Hsu (2015)
  PCA9685 port, safety interlocks and serial interface: adapted with AI assistance (2026).
  This file preserves the V4 inverse-kinematics equations and movement sequences.

  DISCLAIMER: software adaptation only. Hardware operation is NOT validated.
  All 12 servo calibrations intentionally UNSET. The robot will not ARM
  until the real, measured calibration values are entered.

  Wiring: Nano 5V->PCA VCC (logic only), GND->GND, A4->SDA, A5->SCL.
  Servo power: separate regulated supply to PCA V+, COMMON GROUND.
  NEVER power 12 motors from Nano 5V or USB. PCA OE is not controlled here.

  Libraries: Wire (Arduino), Adafruit PWM Servo Driver (Adafruit).
  Adafruit BusIO is installed as a dependency by Arduino Library Manager.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

const uint8_t PCA_ADDRESS = 0x40;  // verify with an I2C scanner if needed
const float SERVO_PWM_HZ = 50.0f; // analog servos: common initial rate
const uint16_t UPDATE_PERIOD_MS = 20;
const unsigned long REACH_TIMEOUT_MS = 15000UL;

// Optional V4 demonstration motions (wave/shake/dance) may exceed 0..180 deg.
// Unlock only after checking every requested pose, with real servo calibration.
const bool ENABLE_EXTREME_ANIMATIONS = false;

// V4 logical joint order is {alpha, beta, gamma}.
// Assumed physical wiring per leg is {proximal gamma, intermediate alpha,
// distal beta}, hence {channel + 1, channel + 2, channel + 0}.
// Change this table if inspection of the mechanics reveals another order.
const uint8_t SERVO_CHANNEL[4][3] = {
  {1, 2, 0},  // leg 0: user's assumed front-left
  {4, 5, 3},  // leg 1: user's assumed front-right
  {7, 8, 6},  // leg 2: user's assumed rear-left
  {10, 11, 9} // leg 3: user's assumed rear-right
};

struct ServoCalibration {
  uint16_t usAt0;      // measured pulse for original logical angle 0 deg
  uint16_t usAt180;    // measured pulse for original logical angle 180 deg
  float angleMin;      // measured allowed logical minimum
  float angleMax;      // measured allowed logical maximum
  bool reverse;        // invert the pulse mapping if physical mounting requires it
  bool calibrated;     // MUST become true only after actual measurement
};

// Configured in PHYSICAL channel order PWM0..PWM11. ZERO = NOT CALIBRATED.
// 0 and 180 are calibration reference ANGLES, not guaranteed physical travel.
// Do NOT mark calibrated=true until measured and mechanically checked.
const ServoCalibration SERVO_CAL[12] = {
  {0, 0, 0, 0, false, false}, // PWM0  front-left  proximal MG90
  {0, 0, 0, 0, false, false}, // PWM1  front-left  intermediate SG90
  {0, 0, 0, 0, false, false}, // PWM2  front-left  distal SG90
  {0, 0, 0, 0, false, false}, // PWM3  front-right proximal MG90
  {0, 0, 0, 0, false, false}, // PWM4  front-right intermediate SG90
  {0, 0, 0, 0, false, false}, // PWM5  front-right distal SG90
  {0, 0, 0, 0, false, false}, // PWM6  rear-left   proximal MG90
  {0, 0, 0, 0, false, false}, // PWM7  rear-left   intermediate SG90
  {0, 0, 0, 0, false, false}, // PWM8  rear-left   distal SG90
  {0, 0, 0, 0, false, false}, // PWM9  rear-right  proximal MG90
  {0, 0, 0, 0, false, false}, // PWM10 rear-right  intermediate SG90
  {0, 0, 0, 0, false, false}  // PWM11 rear-right  distal SG90
};

// Deliberately conservative first CALIBRATION TEST window, not servo specs.
// Widen only after an unloaded mechanical inspection and measurement.
const uint16_t TEST_CENTER_US = 1500;
const uint16_t TEST_MIN_US = 1200;
const uint16_t TEST_MAX_US = 1800;
const uint16_t TEST_STEP_MAX_US = 25;

Adafruit_PWMServoDriver pwm(PCA_ADDRESS);
bool pcaReady = false;
bool armed = false;
bool motionFault = false;
int8_t testedChannel = -1;
uint16_t lastTestUs = 0;
unsigned long lastServiceMs = 0;

/* Robot V4 original size and movement settings (verify printed geometry) */
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;

float site_now[4][3];
float site_expect[4][3];
float temp_speed[4][3];
float move_speed = 1;
float speed_multiple = 1;
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
unsigned long rest_counter = 0;
const float KEEP = 255;
const float pi = 3.1415926;

// V4 turning geometry, unchanged.
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

// Prototypes: explicit for compatibility with ordinary C++ compilation.
void servo_attach(void);
void servo_detach(void);
void sit(void);
void stand(void);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void body_left(int i);
void body_right(int i);
void hand_wave(int i);
void hand_shake(int i);
void head_up(int i);
void head_down(int i);
void body_dance(int i);
void servo_service(void);
void set_site(int leg, float x, float y, float z);
void wait_reach(int leg);
void wait_all_reach(void);
bool cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z);
bool polar_to_servo(int leg, float alpha, float beta, float gamma, uint16_t pulses[3]);
void checkService(void);
void disableAllChannels(void);
void tripFault(const __FlashStringHelper *msg);
void pollAbortDuringMotion(void);
void processSerial(void);

bool goodFloat(float x) { return !isnan(x) && !isinf(x); }

void disableAllChannels(void) {
  if (!pcaReady) return;
  for (uint8_t channel = 0; channel < 16; channel++) {
    pwm.setPWM(channel, 0, 4096); // PCA9685 FULL_OFF bit
  }
}

void tripFault(const __FlashStringHelper *msg) {
  if (!motionFault) {
    Serial.print(F("FAULT: "));
    Serial.println(msg);
  }
  armed = false;
  motionFault = true;
  testedChannel = -1;
  lastTestUs = 0;
  disableAllChannels();
}

bool calibrationValid(uint8_t channel) {
  const ServoCalibration &c = SERVO_CAL[channel];
  // Electrical plausibility guard only, NOT mechanical safety calibration.
  return c.calibrated && c.usAt0 >= 750 && c.usAt0 <= 2250 &&
         c.usAt180 >= 750 && c.usAt180 <= 2250 &&
         c.usAt0 != c.usAt180 && c.angleMin >= 0.0f &&
         c.angleMax <= 180.0f && c.angleMax > c.angleMin;
}

bool channelMapValid(void) {
  bool seen[12] = {false};
  for (uint8_t leg = 0; leg < 4; leg++) {
    for (uint8_t joint = 0; joint < 3; joint++) {
      uint8_t ch = SERVO_CHANNEL[leg][joint];
      if (ch >= 12 || seen[ch]) return false;
      seen[ch] = true;
    }
  }
  return true;
}

bool allCalibrated(void) {
  for (uint8_t c = 0; c < 12; c++) {
    if (!calibrationValid(c)) return false;
  }
  return true;
}

bool angleToPulse(uint8_t channel, float degrees, uint16_t &microseconds) {
  if (channel >= 12 || !calibrationValid(channel) || !goodFloat(degrees)) return false;
  const ServoCalibration &c = SERVO_CAL[channel];
  if (degrees < c.angleMin - 0.001f || degrees > c.angleMax + 0.001f) return false;
  // Same degree domain as original Servo.write(), now mapped to measured pulses.
  float fraction = degrees / 180.0f;
  if (c.reverse) fraction = 1.0f - fraction;
  float pulse = c.usAt0 + (c.usAt180 - c.usAt0) * fraction;
  if (!goodFloat(pulse) || pulse < 750 || pulse > 2250) return false;
  microseconds = (uint16_t)(pulse + 0.5f);
  return true;
}

// Deliver a COMPLETE frame only after verifying all 12 requested angles.
bool computeFrame(const float locations[4][3], uint16_t frame[12]) {
  for (uint8_t leg = 0; leg < 4; leg++) {
    float a, b, g;
    if (!cartesian_to_polar(a, b, g, locations[leg][0], locations[leg][1], locations[leg][2])) return false;
    uint16_t pulses[3];
    if (!polar_to_servo(leg, a, b, g, pulses)) return false;
    for (uint8_t joint = 0; joint < 3; joint++) {
      frame[SERVO_CHANNEL[leg][joint]] = pulses[joint];
    }
  }
  return true;
}

void emitFrame(const uint16_t frame[12]) {
  for (uint8_t channel = 0; channel < 12; channel++) {
    // writeMicroseconds uses the PCA9685 oscillator assumption; its absolute
    // accuracy has to be checked on the real board during calibration.
    pwm.writeMicroseconds(channel, frame[channel]);
  }
}

void servo_attach(void) { // V4 API retained; explicit ARM now controls actual enable
  if (!pcaReady || motionFault || armed) return;
  if (!channelMapValid()) {
    Serial.println(F("LOCKED: invalid or duplicate PCA9685 channel mapping."));
    return;
  }
  if (!allCalibrated()) {
    Serial.println(F("LOCKED: enter 12 real servo calibrations first."));
    return;
  }
  uint16_t bootFrame[12];
  if (!computeFrame(site_now, bootFrame)) {
    Serial.println(F("LOCKED: boot pose outside calibrated joint limits or IK."));
    return;
  }
  disableAllChannels();
  testedChannel = -1;
  lastTestUs = 0;
  // Explicit operator ARM is necessary: output enable can MOVE a real servo.
  armed = true;
  emitFrame(bootFrame);
  lastServiceMs = millis();
  Serial.println(F("ARMED at verified boot setpoint (actual posture NOT sensed)."));
}

void servo_detach(void) {
  armed = false;
  testedChannel = -1;
  lastTestUs = 0;
  disableAllChannels();
}

void printHelp(void) {
  Serial.println(F("Commands: HELP, STATUS, OFF, ARM"));
  Serial.println(F("  TEST channel pulse_us    (single unloaded servo, first 1500us)"));
  Serial.println(F("  STAND, SIT, FWD n, BACK n, LEFT n, RIGHT n"));
  Serial.println(F("  BODYLEFT n, BODYRIGHT n, HEADUP n, HEADDOWN n"));
  Serial.println(F("  WAVE n, SHAKE n, DANCE n [extra lock]"));
  Serial.println(F("  !  = emergency software stop during movement"));
  Serial.println(F("No automatic movement; motor power and calibration required."));
}

void printStatus(void) {
  Serial.print(F("PCA9685: ")); Serial.println(pcaReady ? F("OK") : F("NOT FOUND"));
  Serial.print(F("Channel map valid: ")); Serial.println(channelMapValid() ? F("YES") : F("NO"));
  Serial.print(F("All 12 calibrated: ")); Serial.println(allCalibrated() ? F("YES") : F("NO"));
  Serial.print(F("Motion armed: ")); Serial.println(armed ? F("YES") : F("NO"));
  Serial.print(F("Fault latched: ")); Serial.println(motionFault ? F("YES") : F("NO"));
  Serial.println(F("Logical channels: alpha/beta/gamma = {1,2,0},{4,5,3},{7,8,6},{10,11,9}"));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Original V4 boot-site placement; no servo output at startup.
  const float initial[4][3] = {
    {x_default - x_offset, y_start + y_step, z_boot},
    {x_default - x_offset, y_start + y_step, z_boot},
    {x_default + x_offset, y_start, z_boot},
    {x_default + x_offset, y_start, z_boot}
  };
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 3; j++) {
      site_now[i][j] = initial[i][j];
      site_expect[i][j] = initial[i][j];
      temp_speed[i][j] = 0;
    }
  Wire.beginTransmission(PCA_ADDRESS);
  if (Wire.endTransmission() != 0 || !pwm.begin()) {
    Serial.println(F("PCA9685 missing: check Nano A4/A5, VCC, GND and address."));
    return;
  }
  pcaReady = true;
  pwm.setPWMFreq(SERVO_PWM_HZ);
  delay(10);
  disableAllChannels();
  lastServiceMs = millis();
  Serial.println(F("Spider V4 PCA9685 firmware initialized. Outputs OFF."));
  printStatus();
  printHelp();
}

void pollAbortDuringMotion(void) {
  // Only '!' is acted upon during blocking V4 movements; other bytes discarded.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '!') {
      tripFault(F("operator emergency stop"));
      return;
    }
  }
}

void checkService(void) {
  if (!armed || motionFault) return;
  unsigned long now = millis();
  if ((unsigned long)(now - lastServiceMs) < UPDATE_PERIOD_MS) return;
  lastServiceMs = now;
  servo_service();
}

void doTest(int channel, int pulseUs) {
  if (!pcaReady || armed || motionFault) {
    Serial.println(F("TEST refused: use OFF first and check connection.")); return;
  }
  if (channel < 0 || channel >= 12) { Serial.println(F("Invalid channel 0..11.")); return; }
  if (pulseUs < TEST_MIN_US || pulseUs > TEST_MAX_US) {
    Serial.println(F("TEST refused: outside conservative test interval.")); return;
  }
  if (testedChannel != channel) {
    if (pulseUs != TEST_CENTER_US) {
      Serial.println(F("First TEST on a channel must use 1500 us.")); return;
    }
    disableAllChannels(); // only ONE channel can ever be manually driven
  } else if (abs(pulseUs - (int)lastTestUs) > (int)TEST_STEP_MAX_US) {
    Serial.println(F("TEST refused: change no more than 25 us per command.")); return;
  }
  testedChannel = channel;
  lastTestUs = pulseUs;
  pwm.writeMicroseconds((uint8_t)channel, (uint16_t)pulseUs);
  Serial.print(F("Manual PWM")); Serial.print(channel);
  Serial.print(F(" = ")); Serial.print(pulseUs); Serial.println(F(" us; others OFF."));
}

void executeCommand(char *line) {
  char *cmd = strtok(line, " ,\t");
  if (!cmd) return;
  // Arduino serial monitor can send lower-case; normalize command only.
  for (char *p = cmd; *p; ++p) if (*p >= 'a' && *p <= 'z') *p -= ('a' - 'A');
  if (strcmp(cmd, "OFF") == 0) {
    servo_detach(); motionFault = false;
    Serial.println(F("All PWM outputs OFF. Fault cleared.")); return;
  }
  if (strcmp(cmd, "HELP") == 0) { printHelp(); return; }
  if (strcmp(cmd, "STATUS") == 0) { printStatus(); return; }
  char *arg1 = strtok(NULL, " ,\t");
  if (strcmp(cmd, "TEST") == 0) {
    char *arg2 = strtok(NULL, " ,\t");
    if (!arg1 || !arg2) { Serial.println(F("Usage: TEST channel pulse_us")); return; }
    doTest(atoi(arg1), atoi(arg2)); return;
  }
  if (strcmp(cmd, "ARM") == 0) { servo_attach(); return; }
  if (!armed || motionFault) {
    Serial.println(F("LOCKED: calibrate channels, ensure external power, then ARM.")); return;
  }
  // Limit user-requested repetitions, while keeping the original V4 gaits.
  int n = arg1 ? atoi(arg1) : 1;
  if (n < 1 || n > 10) { Serial.println(F("Argument n must be 1..10.")); return; }
  if (strcmp(cmd, "STAND") == 0) stand();
  else if (strcmp(cmd, "SIT") == 0) sit();
  else if (strcmp(cmd, "FWD") == 0) step_forward((unsigned int)n);
  else if (strcmp(cmd, "BACK") == 0) step_back((unsigned int)n);
  else if (strcmp(cmd, "LEFT") == 0) turn_left((unsigned int)n);
  else if (strcmp(cmd, "RIGHT") == 0) turn_right((unsigned int)n);
  else if (strcmp(cmd, "BODYLEFT") == 0) {move_speed = body_move_speed; body_left(n);}
  else if (strcmp(cmd, "BODYRIGHT") == 0) {move_speed = body_move_speed; body_right(n);}
  else if (strcmp(cmd, "HEADUP") == 0) {move_speed = body_move_speed; head_up(n);}
  else if (strcmp(cmd, "HEADDOWN") == 0) {move_speed = body_move_speed; head_down(n);}
  else if (strcmp(cmd, "WAVE") == 0 || strcmp(cmd, "SHAKE") == 0 || strcmp(cmd, "DANCE") == 0) {
    if (!ENABLE_EXTREME_ANIMATIONS) Serial.println(F("Animation locked until physical pose validation."));
    else if (strcmp(cmd, "WAVE") == 0) hand_wave(n);
    else if (strcmp(cmd, "SHAKE") == 0) hand_shake(n);
    else body_dance(n);
  }
  else Serial.println(F("Unknown command; type HELP."));
  if (motionFault) Serial.println(F("Movement aborted. Fix cause, then OFF / ARM."));
}

void processSerial(void) {
  static char line[44];
  static uint8_t count = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '!') { tripFault(F("operator emergency stop")); count = 0; return; }
    if (c == '\r' || c == '\n') {
      if (count > 0) { line[count] = '\0'; executeCommand(line); count = 0; }
    } else if (count < sizeof(line) - 1) {
      line[count++] = c;
    } else {
      count = 0;
      Serial.println(F("Command too long; buffer cleared."));
    }
  }
}

void loop() {
  checkService();
  processSerial();
}

// -----------------------------------------------------------------------------
// ORIGINAL V4 MOVEMENT ROUTINES START: preserved as written in spider_open_v4.
// (They still use set_site() / wait_all_reach(), now serviced in main context.)
// -----------------------------------------------------------------------------
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu

void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}

void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void body_dance(int i)
{
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  //stand();
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
}



// -----------------------------------------------------------------------------
// PCA9685 ADAPTATION: 20 ms main-loop servo interpolation, no Timer2 ISR/I2C ISR.
// -----------------------------------------------------------------------------
#define E_DELTA 0.01f
void servo_service(void) {
  if (!armed || motionFault) return;
  float nextSites[4][3];
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      float delta = site_expect[i][j] - site_now[i][j];
      if (fabs(delta) <= fabs(temp_speed[i][j]) + E_DELTA)
        nextSites[i][j] = site_expect[i][j];
      else
        nextSites[i][j] = site_now[i][j] + temp_speed[i][j];
    }
  }
  uint16_t frame[12];
  if (!computeFrame(nextSites, frame)) {
    tripFault(F("IK invalid or servo outside calibrated safe range"));
    return;
  }
  // The logical state is updated only after a valid full 12-servo frame.
  emitFrame(frame); // Adafruit writeMicroseconds() does not report I2C errors.
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 3; j++) site_now[i][j] = nextSites[i][j];
  rest_counter++;
}

void set_site(int leg, float x, float y, float z) {
  if (motionFault || leg < 0 || leg >= 4) return;
  float dx = (x == KEEP) ? 0.0f : x - site_now[leg][0];
  float dy = (y == KEEP) ? 0.0f : y - site_now[leg][1];
  float dz = (z == KEEP) ? 0.0f : z - site_now[leg][2];
  float dist = sqrt(dx * dx + dy * dy + dz * dz);
  if (!goodFloat(dist) || !goodFloat(move_speed) || move_speed <= 0 || !goodFloat(speed_multiple) || speed_multiple <= 0) {
    tripFault(F("Invalid target or movement speed")); return;
  }
  if (dist <= 0.0001f) {
    temp_speed[leg][0] = 0;
    temp_speed[leg][1] = 0;
    temp_speed[leg][2] = 0;
  } else {
    float scale = move_speed * speed_multiple / dist;
    temp_speed[leg][0] = dx * scale;
    temp_speed[leg][1] = dy * scale;
    temp_speed[leg][2] = dz * scale;
  }
  if (x != KEEP) site_expect[leg][0] = x;
  if (y != KEEP) site_expect[leg][1] = y;
  if (z != KEEP) site_expect[leg][2] = z;
}

void wait_reach(int leg) {
  unsigned long started = millis();
  while (armed && !motionFault) {
    bool reached = true;
    for (uint8_t j = 0; j < 3; j++)
      if (fabs(site_now[leg][j] - site_expect[leg][j]) > E_DELTA) reached = false;
    if (reached) return;
    checkService();
    pollAbortDuringMotion();
    if ((unsigned long)(millis() - started) > REACH_TIMEOUT_MS) {
      tripFault(F("Timed out waiting for target")); return;
    }
    delay(1);
  }
}

void wait_all_reach(void) {
  for (uint8_t leg = 0; leg < 4 && armed && !motionFault; leg++) wait_reach(leg);
}

// V4 inverse kinematics, same geometric equations with domain checks around acos.
bool cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z) {
  if (!goodFloat(x) || !goodFloat(y) || !goodFloat(z)) return false;
  float w = (x >= 0 ? 1 : -1) * sqrt(x*x + y*y);
  float v = w - length_c;
  float r = sqrt(v*v + z*z);
  if (!goodFloat(r) || r < 0.0001f) return false;
  float cosA = (length_a*length_a - length_b*length_b + v*v + z*z) / (2 * length_a * r);
  float cosB = (length_a*length_a + length_b*length_b - v*v - z*z) / (2 * length_a * length_b);
  // Allow tiny floating-point rounding error; reject geometrically unreachable poses.
  if (!goodFloat(cosA) || !goodFloat(cosB) || cosA < -1.00001f || cosA > 1.00001f ||
      cosB < -1.00001f || cosB > 1.00001f) return false;
  cosA = constrain(cosA, -1.0f, 1.0f);
  cosB = constrain(cosB, -1.0f, 1.0f);
  alpha = (atan2(z, v) + acos(cosA)) / pi * 180;
  beta  = acos(cosB) / pi * 180;
  gamma = ((w >= 0) ? atan2(y, x) : atan2(-y, -x)) / pi * 180;
  return goodFloat(alpha) && goodFloat(beta) && goodFloat(gamma);
}

// V4 polar joint-angle transforms unchanged; only physical output is replaced.
bool polar_to_servo(int leg, float alpha, float beta, float gamma, uint16_t pulses[3]) {
  if (leg == 0) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  } else if (leg == 1) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 2) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 3) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  } else return false;

  const float angles[3] = { alpha, beta, gamma };
  for (uint8_t joint = 0; joint < 3; joint++) {
    if (!angleToPulse(SERVO_CHANNEL[leg][joint], angles[joint], pulses[joint])) return false;
  }
  return true;
}
