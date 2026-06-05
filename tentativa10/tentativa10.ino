// ============================================================
//  Assessment Tests – physical measurement protocol
//  Builds on the Calibration / Compression base code.
//  Firmware executes repeatable actuator motions only.
//  Actual displacement, error, deviation, and standard deviation
//  are measured externally with microscope/reference scale.
//  Tests are manual step-by-step: user taps NEXT to advance.
// ============================================================

#include <Arduino_GFX_Library.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>

// =============== PINS ===============
#define PIN_DIR  4
#define PIN_STEP 3
#define PIN_EN   2

#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8
#define TOUCH_CS 7

#define TFT_WIDTH  320
#define TFT_HEIGHT 240

// =============== TOUCH CALIBRATION ===============
#define TS_MINX 100
#define TS_MAXX 3800
#define TS_MINY 280
#define TS_MAXY 3800

// =============== MOTOR PARAMETERS ===============
#define FULL_STEPS_PER_REV 200
#define MICROSTEP          8
#define LEAD_MM            4.35f

const long  STEPS_PER_REV_MICRO = (long)FULL_STEPS_PER_REV * (long)MICROSTEP;
const float STEPS_PER_MM_MICRO  = (float)STEPS_PER_REV_MICRO / LEAD_MM;

// =============== RPM CONTROL ===============
const float RPM_NORMAL    = 200.0f;
const float RPM_RETRACT   = 80.0f;
const float RPM_SLOW      = 25.0f;
const float RPM_ALIGNMENT = 50.0f;

// ====================== DISTANCE PARAMETERS (mm) ======================
const float PREPARE_DISTANCE_MM  = 15.7f;
const float COMPRESS_DISTANCE_MM = 1.0f;
const float COMPRESS_RETRACT_MM  = 2.0f;
const float MAX_DISTANCE_MM      = 40.0f;
const float TEST_DISTANCE_MIN    = 0.0f;
const float TEST_DISTANCE_MAX    = 17.0f;

// ====================== COLOR PALETTE ======================
#define RGB565(r,g,b) (((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | ((uint16_t)(b) >> 3))

#define BLACK      RGB565(0,   0,   0)
#define WHITE      RGB565(255, 255, 255)
#define RED        RGB565(255, 0,   0)
#define GREEN      RGB565(0,   200, 0)
#define ORANGE     RGB565(255, 165, 0)
#define GREY       RGB565(128, 128, 128)
#define LIGHTGREY  RGB565(192, 192, 192)
#define DARKGREY   RGB565(64,  64,  64)
#define LIGHTGREEN RGB565(144, 238, 144)
#define TEAL       RGB565(0,   128, 128)
#define GOLD       RGB565(255, 215, 0)
#define CYAN       RGB565(0,   230, 230)
#define MAROON     RGB565(128, 0,   0)

// =============== MENU STATES ===============
enum MenuState {
  MAIN_MENU,
  CALIBRATION_MENU,
  COMPRESSION_MENU,
  ADMIN_PASSWORD_MENU,
  ASSESSMENT_MENU,
  ASSESSMENT_DETAIL,
  ASSESSMENT_RUN
};

// ====================== ASSESSMENT TEST IDs ======================
#define NUM_TESTS               8
#define TEST_ALIGNMENT          0
#define TEST_POSITIONING        1
#define TEST_REPEATABILITY      2
#define TEST_ZERO_REPEATABILITY 3
#define TEST_BIDIRECTIONAL      4
#define TEST_DUTY_CYCLE         5
#define TEST_CYCLIC             6
#define TEST_ENDURANCE          7

// ====================== TEST RESULT STORAGE ======================
#define MAX_REPS 20
#define NUM_POS_TARGETS 9

const float TEST_PREPARE_MM     = PREPARE_DISTANCE_MM;
const float TEST_COMPRESSION_MM = PREPARE_DISTANCE_MM + COMPRESS_DISTANCE_MM;

struct TestResults {
  float measured[MAX_REPS];
  float measuredFwd[MAX_REPS];
  float measuredRev[MAX_REPS];
  int   repsTotal;
  int   repsDone;
  float mean;
  float maxDev;
  float stdDev;
  float posTargets[NUM_POS_TARGETS];
  float posMeasured[NUM_POS_TARGETS][5];
  float posMeanErr[NUM_POS_TARGETS];
  float posMaxErr[NUM_POS_TARGETS];
  float tempMotorStart;
  float tempDriverStart;
  float tempMotorEnd;
  float tempDriverEnd;
};

TestResults tr;

// ====================== TEST EXECUTION STATE ======================
enum TestPhase {
  TP_IDLE,
  TP_INTRO,
  TP_MOVING,
  TP_AWAIT_TAP,
  TP_RESULTS
};

struct TestRunState {
  TestPhase phase       = TP_IDLE;
  int       currentRep  = 0;
  int       subStep     = 0;
  int       targetIdx   = 0;
};
TestRunState trs;

// ====================== ADMIN ACCESS ======================
const char ADMIN_PASSWORD[] = "123";
char adminPasswordInput[4] = "";
int  adminPasswordLen = 0;
bool adminPasswordError = false;

// ====================== METADATA ======================
const char* testNames[NUM_TESTS] = {
  "Alignment", "Positioning", "Repeatability", "Zero Repeat.",
  "Bidirectional", "Duty Cycle", "Cyclic", "Endurance"
};

const char* testTitles[NUM_TESTS] = {
  "Alignment", "Positioning Accuracy", "Repeatability",
  "Zero Repeatability", "Bidirectional/Backlash",
  "Mechanical Duty Cycle", "Cyclic Test", "Endurance"
};

const char* testObjectives[NUM_TESTS] = {
  "Verify linear trajectory\nwithout lateral deviation.",
  "Evaluate commanded vs\nactual displacement.",
  "Evaluate positioning\nconsistency under\nidentical commands.",
  "Evaluate consistency\nof return to zero.",
  "Evaluate direction-dependent\nerrors: backlash & hysteresis.",
  "Evaluate thermal behavior\nunder repeated short-term\noperation.",
  "Evaluate consistency\nduring repeated full\noperation cycles.",
  "Evaluate long-term\nmechanical & functional\nrobustness."
};

const char* testProcedure[NUM_TESTS] = {
  "- Slow motion over full range\n- Observe lateral deviation\n- 5 cycles",
  "- Command known displacements\n  0.05, 0.10, 0.25,\n  0.50, 1.00, 1.50,\n  2.00, 2.50, 3.00 mm\n- 5 reps per distance",
  "- Move 0->0.50mm->zero\n- 10 repetitions",
  "- Move 5mm away from zero\n- Return to zero\n- 10 repetitions",
  "- Approach 0.50mm from below\n- Approach from above\n- 10 cycles",
  "- 300 full cycles:\n  zero->Prepare->Compression->zero\n- Pause every 15 cycles\n- Record motor/driver temperature",
  "- zero->prepare->retract\n  ->compress\n- Stop for measurement\n- Return zero on NEXT",
  "- 75 continuous cycles\n- zero->prepare->retract\n  ->compress->zero\n- Inspect integrity"
};

const char* testVariables[NUM_TESTS] = {
  "Lateral deviation (qualitative),\nalignment, trajectory stability",
  "Commanded vs measured (mm),\nabsolute error per distance",
  "Final position per rep (mm),\nstd dev & max deviation",
  "Position at zero (mm),\nmax & mean deviation",
  "Forward & reverse pos (mm),\ndifference per cycle",
  "Temperature (C) motor+driver,\nrise over test",
  "Compressed position per cycle,\ndrift & failure occurrence",
  "Repeatability & backlash\nbefore vs after, degradation"
};

const uint16_t testColors[NUM_TESTS] = {
  RGB565(0,   120, 180), RGB565(0,   150, 100),
  RGB565(140, 80,  180), RGB565(180, 100, 0),
  RGB565(180, 50,  50),  RGB565(0,   130, 130),
  RGB565(100, 140, 0),   RGB565(120, 60,  120)
};

// =============== UI CONFIG ===============
#define SCREEN_CENTER_X   (TFT_WIDTH / 2)
#define UI_LINE_THICKNESS 2
#define BTN_RADIUS        20
#define BTN_SHADOW_OFS    2
#define BTN_SHADOW_COLOR  DARKGREY
#define BTN_TEXT_SIZE_SMALL 2
#define BTN_TEXT_SIZE_LARGE 3

#define MAIN_LINE_Y    85
#define CAL_LINE_Y     35
#define COMP_LINE_Y    50
#define COMP_TITLE_Y   25
#define CAL_STATUS_Y   50
#define COMP_STATUS_Y  75
#define COMP_BUTTONS_Y 115

#define STATUS_CLEAR_H    32
#define STATUS_CLEAR_PAD   8
#define MESSAGE_DURATION         500
#define STATUS_MESSAGE_DURATION  2000
#define FINISHED_DISPLAY_DURATION 500

// ====================== BUTTON ======================
struct RectButton {
  int16_t x, y, w, h;
  uint16_t color;
  const char *label;
};

// ====================== STATE ======================
struct MotorState {
  long currentPosSteps = 0;
  long zeroOffsetSteps = 0;
  bool zeroDefined     = false;
  volatile bool stopFlag = false;
  bool isPrepared      = false;
  bool mustClickZero   = false;
};

struct UIState {
  MenuState currentMenu = MAIN_MENU;
  bool showZeroMessage  = false;
  unsigned long messageStartTime = 0;
  unsigned long statusMessageStartTime = 0;
  bool pendingPostCompress = false;
  unsigned long finishedMessageTime = 0;
  int  previousZeroState            = -1;
  int  previousCompressionZeroState = -1;
  bool previousPrepareEnabled       = false;
  bool previousCompressEnabled      = false;
  bool compressionMenuEntered       = false;
  bool isTesting     = false;
  bool isPreparing   = false;
  bool isCompressing = false;
  int  selectedTest  = -1;
};

struct JogState {
  unsigned long periodUs     = 0;
  unsigned long lastStepTime = 0;
  bool active    = false;
  bool direction = true;
};

MotorState motor;
UIState    ui;
JogState   jog;

Arduino_DataBus    *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
Arduino_GFX        *gfx = new Arduino_ILI9341(bus, TFT_RST, 0, false);
XPT2046_Touchscreen ts(TOUCH_CS);

// =============================================================
//  UTILITY
// =============================================================
void drawCenteredText(const char *text, int16_t cx, int16_t y,
                      uint16_t color, uint8_t size) {
  gfx->setTextSize(size); gfx->setTextColor(color);
  int16_t tbx, tby; uint16_t tbw, tbh;
  gfx->getTextBounds(text, 0, 0, &tbx, &tby, &tbw, &tbh);
  gfx->setCursor(cx - tbw/2, y); gfx->print(text);
}

void drawThickHLine(int16_t x, int16_t y, int16_t w, int16_t t, uint16_t c) {
  gfx->fillRect(x, y, w, t, c);
}

void printMultiLine(const char *text, int16_t x, int16_t y,
                    uint16_t color, uint8_t size, uint8_t spacing) {
  gfx->setTextColor(color); gfx->setTextSize(size);
  char buf[160]; strncpy(buf, text, 159); buf[159] = '\0';
  char *line = strtok(buf, "\n"); int16_t cy = y;
  while (line) {
    gfx->setCursor(x, cy); gfx->print(line);
    cy += spacing; line = strtok(nullptr, "\n");
  }
}

float stepsToMm(long steps) { return (float)steps / STEPS_PER_MM_MICRO; }
float currentPosMm()        { return stepsToMm(motor.currentPosSteps - motor.zeroOffsetSteps); }

// =============================================================
//  BUTTONS
// =============================================================
void drawButton(const RectButton &btn, bool enabled = true) {
  uint16_t bg = enabled ? btn.color : DARKGREY;
  uint16_t fg = enabled ? WHITE     : GREY;
  gfx->fillRoundRect(btn.x+BTN_SHADOW_OFS, btn.y+BTN_SHADOW_OFS,
                     btn.w, btn.h, BTN_RADIUS, BTN_SHADOW_COLOR);
  gfx->fillRoundRect(btn.x, btn.y, btn.w, btn.h, BTN_RADIUS, bg);
  uint8_t sz = BTN_TEXT_SIZE_SMALL;
  if (btn.h >= 50 && (strcmp(btn.label,"Test")==0 ||
      strcmp(btn.label,"Prepare")==0 || strcmp(btn.label,"Compress")==0))
    sz = BTN_TEXT_SIZE_LARGE;
  gfx->setTextSize(sz); gfx->setTextColor(fg);
  int16_t tbx, tby; uint16_t tbw, tbh;
  gfx->getTextBounds(btn.label, 0, 0, &tbx, &tby, &tbw, &tbh);
  gfx->setCursor(btn.x+(btn.w-tbw)/2, btn.y+(btn.h-tbh)/2);
  gfx->print(btn.label);
}

bool hitRect(int16_t px, int16_t py, const RectButton &btn) {
  return px>=btn.x && px<btn.x+btn.w && py>=btn.y && py<btn.y+btn.h;
}

// =============================================================
//  STATUS HELPERS
// =============================================================
void updateStatusText(int16_t sy, const char *text, uint16_t color) {
  gfx->fillRect(0, sy-STATUS_CLEAR_PAD, TFT_WIDTH, STATUS_CLEAR_H, BLACK);
  drawCenteredText(text, SCREEN_CENTER_X, sy, color, BTN_TEXT_SIZE_SMALL);
}

void updateZeroStatusText() {
  const char *t = "No zero set"; uint16_t c = LIGHTGREY;
  if (motor.zeroDefined && ui.showZeroMessage) { t="Zero Defined!"; c=ORANGE; }
  else if (motor.zeroDefined)                  { t="Ready";         c=GREEN;  }
  updateStatusText(CAL_STATUS_Y, t, c);
}

void updateCompressionDisplayStatus() {
  const char *t = "Define Zero in Calibration"; uint16_t c = RED;
  if (motor.mustClickZero)    { t="Click in Zero"; c=ORANGE; }
  else if (motor.zeroDefined) { t="Ready";         c=GREEN;  }
  updateStatusText(COMP_STATUS_Y, t, c);
}

// =============================================================
//  MOTOR
// =============================================================
unsigned long rpmToPeriodUs(float rpm) {
  float sps = (rpm * STEPS_PER_REV_MICRO) / 60.0f;
  if (sps < 1.0f) sps = 1.0f;
  return (unsigned long)(1000000.0f / sps);
}

void stepMotor(bool fwd) {
  digitalWrite(PIN_DIR, fwd ? HIGH : LOW);
  digitalWrite(PIN_STEP, HIGH); delayMicroseconds(1);
  digitalWrite(PIN_STEP, LOW);
}

void enableMotor(bool en) { digitalWrite(PIN_EN, en ? LOW : HIGH); }

long moveMotorBlocking(long steps, unsigned long periodUs) {
  if (steps==0) return 0;
  bool fwd = (steps>0);
  digitalWrite(PIN_DIR, fwd ? HIGH : LOW);
  enableMotor(true);
  long count = labs(steps);
  for (long i=0; i<count; i++) {
    if (motor.stopFlag) { enableMotor(false); return fwd?i:-i; }
    stepMotor(fwd);
    if (periodUs>2) delayMicroseconds(periodUs-2);
  }
  enableMotor(false);
  return fwd ? count : -count;
}

long mmToSteps(float mm) { return (long)lroundf(mm * STEPS_PER_MM_MICRO); }

long getPreparePositionSteps() {
  long p = motor.zeroOffsetSteps + mmToSteps(PREPARE_DISTANCE_MM);
  long m = motor.zeroOffsetSteps + mmToSteps(MAX_DISTANCE_MM);
  return (p>m)?m:p;
}

bool isAtPreparePosition() {
  return motor.zeroDefined && (motor.currentPosSteps == getPreparePositionSteps());
}

void goToZero() {
  if (!motor.zeroDefined) return;
  motor.stopFlag = false;
  long delta = motor.zeroOffsetSteps - motor.currentPosSteps;
  if (delta==0) return;
  motor.currentPosSteps += moveMotorBlocking(delta, rpmToPeriodUs(RPM_NORMAL));
}

void defineZero() {
  motor.zeroOffsetSteps = motor.currentPosSteps;
  motor.zeroDefined = true;
  ui.showZeroMessage = true;
  ui.messageStartTime = millis();
}

void moveToMm(float targetMm, float rpm = RPM_NORMAL) {
  if (!motor.zeroDefined) return;
  motor.stopFlag = false;
  long ts = motor.zeroOffsetSteps + mmToSteps(targetMm);
  long maxS = motor.zeroOffsetSteps + mmToSteps(MAX_DISTANCE_MM);
  long minS = motor.zeroOffsetSteps;
  if (ts > maxS) ts = maxS;
  if (ts < minS) ts = minS;
  long delta = ts - motor.currentPosSteps;
  if (delta==0) return;
  motor.currentPosSteps += moveMotorBlocking(delta, rpmToPeriodUs(rpm));
}

void moveRelativeMm(float deltaMm, float rpm = RPM_NORMAL) {
  if (!motor.zeroDefined) return;
  motor.stopFlag = false;
  long deltaSteps = mmToSteps(deltaMm);
  if (deltaSteps == 0 && fabsf(deltaMm) > 0.0f) deltaSteps = (deltaMm > 0.0f) ? 1 : -1;

  long minS = motor.zeroOffsetSteps;
  long maxS = motor.zeroOffsetSteps + mmToSteps(MAX_DISTANCE_MM);
  long target = motor.currentPosSteps + deltaSteps;
  if (target > maxS) target = maxS;
  if (target < minS) target = minS;

  long delta = target - motor.currentPosSteps;
  if (delta == 0) return;
  motor.currentPosSteps += moveMotorBlocking(delta, rpmToPeriodUs(rpm));
}

// =============================================================
//  CALIBRATION TEST (existing)
// =============================================================
void testCalibration() {
  motor.stopFlag = false; ui.isTesting = true;
  ui.statusMessageStartTime = millis();
  updateStatusText(CAL_STATUS_Y, "Testing...", ORANGE);
  for (int i=0; i<3; i++) {
    if (motor.stopFlag) break;
    long d = random((long)TEST_DISTANCE_MIN, (long)TEST_DISTANCE_MAX+1);
    unsigned long p = rpmToPeriodUs(RPM_NORMAL);
    motor.currentPosSteps += moveMotorBlocking( mmToSteps((float)d), p);
    if (motor.stopFlag) break; delay(200);
    motor.currentPosSteps += moveMotorBlocking(-mmToSteps((float)d), p);
    if (motor.stopFlag) break; delay(200);
  }
  goToZero(); ui.isTesting = false;
  ui.statusMessageStartTime = millis();
  updateStatusText(CAL_STATUS_Y, "Ready", GREEN);
}

// =============================================================
//  COMPRESSION (existing)
// =============================================================
void prepareDistancemm() {
  if (isAtPreparePosition()) {
    motor.isPrepared = true; ui.statusMessageStartTime = millis();
    updateStatusText(COMP_STATUS_Y,"Already at position",GREEN); return;
  }
  motor.stopFlag=false; ui.isPreparing=true;
  updateStatusText(COMP_STATUS_Y,"Preparing...",ORANGE);
  motor.currentPosSteps += moveMotorBlocking(
    getPreparePositionSteps()-motor.currentPosSteps, rpmToPeriodUs(RPM_NORMAL));
  ui.isPreparing=false; motor.isPrepared=true;
  ui.statusMessageStartTime=millis();
  updateStatusText(COMP_STATUS_Y,"Ready to compress",GREEN);
}

void goToPreparePosition() {
  if (!motor.zeroDefined) return;
  if (isAtPreparePosition()) {
    updateStatusText(COMP_STATUS_Y,"Already at position",GREEN);
    ui.statusMessageStartTime=millis(); return;
  }
  motor.stopFlag=false;
  long delta=getPreparePositionSteps()-motor.currentPosSteps;
  if (delta==0) return;
  motor.currentPosSteps+=moveMotorBlocking(delta,rpmToPeriodUs(RPM_NORMAL));
}

void compressDistancemm() {
  motor.stopFlag=false; ui.isCompressing=true;
  updateStatusText(COMP_STATUS_Y,"Compressing...",ORANGE);
  unsigned long pR=rpmToPeriodUs(RPM_RETRACT), pS=rpmToPeriodUs(RPM_SLOW);
  motor.currentPosSteps+=moveMotorBlocking(-mmToSteps(COMPRESS_RETRACT_MM),pR);
  if(motor.stopFlag){ui.isCompressing=false;updateStatusText(COMP_STATUS_Y,"Stopped",RED);return;}
  motor.currentPosSteps+=moveMotorBlocking( mmToSteps(COMPRESS_RETRACT_MM),pR);
  if(motor.stopFlag){ui.isCompressing=false;updateStatusText(COMP_STATUS_Y,"Stopped",RED);return;}
  long tgt=motor.currentPosSteps+mmToSteps(COMPRESS_DISTANCE_MM);
  long maxS=motor.zeroOffsetSteps + mmToSteps(MAX_DISTANCE_MM); if(tgt>maxS)tgt=maxS;
  motor.currentPosSteps+=moveMotorBlocking(tgt-motor.currentPosSteps,pS);
  ui.isCompressing=false; ui.statusMessageStartTime=millis();
  updateStatusText(COMP_STATUS_Y,"Finished",GOLD);
  ui.pendingPostCompress=true; ui.finishedMessageTime=millis();
}


void executeCompressionMovementOnly() {
  // Same movement performed by the normal Compression button.
  // Starting point must already be the prepare position.
  unsigned long pR = rpmToPeriodUs(RPM_RETRACT);
  unsigned long pS = rpmToPeriodUs(RPM_SLOW);

  motor.currentPosSteps += moveMotorBlocking(-mmToSteps(COMPRESS_RETRACT_MM), pR);
  if (motor.stopFlag) return;

  motor.currentPosSteps += moveMotorBlocking( mmToSteps(COMPRESS_RETRACT_MM), pR);
  if (motor.stopFlag) return;

  long tgt  = motor.currentPosSteps + mmToSteps(COMPRESS_DISTANCE_MM);
  long maxS = motor.zeroOffsetSteps + mmToSteps(MAX_DISTANCE_MM);
  if (tgt > maxS) tgt = maxS;
  motor.currentPosSteps += moveMotorBlocking(tgt - motor.currentPosSteps, pS);
}

void executeAssessmentCompressionCycle(bool returnToZeroAfterCompression) {
  // Imitates the normal menu sequence:
  // 1) Zero
  // 2) Press Prepare: move to PREPARE_DISTANCE_MM from zero
  // 3) Press Compression: retract, return to prepare, then compress
  // 4) Optional Zero
  goToZero();
  if (motor.stopFlag) return;
  delay(250);

  long prepDelta = getPreparePositionSteps() - motor.currentPosSteps;
  motor.currentPosSteps += moveMotorBlocking(prepDelta, rpmToPeriodUs(RPM_NORMAL));
  if (motor.stopFlag) return;
  motor.isPrepared = true;
  delay(350);

  executeCompressionMovementOnly();
  if (motor.stopFlag) return;
  delay(350);

  if (returnToZeroAfterCompression) {
    goToZero();
    delay(250);
  }
}

// =============================================================
//  STATISTICS
// =============================================================
void calcStats(float *data, int n, float &mean, float &maxDev, float &stdDev) {
  if (n==0){mean=maxDev=stdDev=0;return;}
  float sum=0; for(int i=0;i<n;i++) sum+=data[i];
  mean=sum/n;
  float ssq=0,md=0;
  for(int i=0;i<n;i++){float d=fabsf(data[i]-mean);if(d>md)md=d;ssq+=d*d;}
  maxDev=md; stdDev=(n>1)?sqrtf(ssq/(n-1)):0;
}

// =============================================================
//  SERIAL OUTPUT
// =============================================================
void serialPrintResults() {
  int t = ui.selectedTest;
  Serial.println(F("=============================="));
  Serial.print(F("TEST PROTOCOL: ")); Serial.println(testTitles[t]);
  Serial.println(F("Use external physical/microscope measurements for result tables."));
  Serial.println(F("Firmware values below are commanded positions only, not measured error."));
  Serial.println(F("------------------------------"));

  if (t == TEST_ALIGNMENT) {
    Serial.print(F("Cycles completed: ")); Serial.println(tr.repsDone);
    Serial.println(F("Record: representative trajectory image and qualitative lateral deviation."));

  } else if (t == TEST_POSITIONING) {
    Serial.println(F("Commanded displacement protocol:"));
    Serial.println(F("Target(mm) | Reps | Action"));
    for (int d=0; d<NUM_POS_TARGETS; d++) {
      Serial.print(tr.posTargets[d], 3); Serial.print(F(" | 5 | zero -> target -> record actual position -> zero"));
      Serial.println();
    }
    Serial.println(F("Calculate absolute error later from physical measurements."));

  } else if (t == TEST_REPEATABILITY) {
    Serial.println(F("10 repetitions: zero -> 0.50 mm -> record final position -> zero."));
    Serial.println(F("Calculate deviation from mean and standard deviation later."));

  } else if (t == TEST_ZERO_REPEATABILITY) {
    Serial.println(F("10 repetitions: move 5.00 mm away -> return zero -> record zero position."));
    Serial.println(F("Calculate zero deviation later."));

  } else if (t == TEST_BIDIRECTIONAL) {
    Serial.println(F("10 cycles: approach 0.50 mm from below and from 1.00 mm."));
    Serial.println(F("Record forward and reverse physical positions externally."));
    Serial.println(F("Calculate bidirectional difference later."));

  } else if (t == TEST_DUTY_CYCLE) {
    Serial.print(F("Cycles completed: ")); Serial.println(tr.repsDone);
    Serial.println(F("Record motor and driver temperatures externally at start, every 15 cycles, and at the end."));

  } else if (t == TEST_CYCLIC) {
    Serial.print(F("Full cycles completed: ")); Serial.println(tr.repsDone);
    Serial.println(F("Each cycle imitates: Zero, Prepare button, Compression button, Zero."));
    Serial.println(F("Record actuator position/failures externally."));

  } else if (t == TEST_ENDURANCE) {
    Serial.print(F("Extended cycles completed: ")); Serial.println(trs.currentRep);
    Serial.println(F("Perform before/after repeatability, bidirectional, and zero-reference checks externally."));
    Serial.println(F("Inspect mechanical integrity and record qualitative degradation."));
  }
  Serial.println(F("=============================="));
}

// =============================================================
//  SCREEN DRAWING – EXISTING MENUS
// =============================================================
void drawMainScreen() {
  gfx->fillScreen(BLACK);
  drawCenteredText("Main Menu", SCREEN_CENTER_X, 50, WHITE, 3);
  drawThickHLine(30, MAIN_LINE_Y, 260, UI_LINE_THICKNESS, ORANGE);
  RectButton btnCalib = {25, 105, 270, 42, GREY, "Calibration"};
  RectButton btnComp  = {25, 158, 270, 42, GREY, "Compression"};

  // Small, low-prominence admin entry point.
  RectButton btnAdmin = {244, 214, 70, 20, GREY, "Adm"};

  drawButton(btnCalib);
  drawButton(btnComp);
  drawButton(btnAdmin);
}

void drawCalibrationScreen() {
  gfx->fillScreen(BLACK);
  drawCenteredText("Calibration", SCREEN_CENTER_X, 10, WHITE, 2);
  drawThickHLine(10, CAL_LINE_Y, 300, UI_LINE_THICKNESS, ORANGE);
  updateZeroStatusText();
  ui.previousZeroState = motor.zeroDefined ? 1 : 0;
  RectButton btnBwd  = {10,  80, 95,  40, GREY,   "Bwd"};
  RectButton btnFwd  = {110, 80, 95,  40, GREY,   "Fwd"};
  RectButton btnDef  = {210, 80, 100, 40, GREY,   "D.Zero"};
  RectButton btnTest = {10, 130, 300, 50, ORANGE, "Test"};
  RectButton btnBack = {10, 195, 70,  40, GREY,   "Back"};
  RectButton btnZero = {240,195, 70,  40, GREY,   "Zero"};
  drawButton(btnFwd); drawButton(btnBwd); drawButton(btnDef);
  drawButton(btnTest); drawButton(btnBack); drawButton(btnZero);
}

void drawCompressionScreen() {
  gfx->fillScreen(BLACK);
  drawCenteredText("Compression", SCREEN_CENTER_X, COMP_TITLE_Y, WHITE, 2);
  drawThickHLine(10, COMP_LINE_Y, 300, UI_LINE_THICKNESS, ORANGE);
  updateCompressionDisplayStatus();
  ui.previousCompressionZeroState = motor.zeroDefined ? 1 : 0;
  ui.previousPrepareEnabled  = (motor.zeroDefined && !motor.mustClickZero);
  ui.previousCompressEnabled = (motor.isPrepared && motor.zeroDefined && !motor.mustClickZero);
  RectButton btnPrepare     = {10,  COMP_BUTTONS_Y, 145, 50, ORANGE,     "Prepare"};
  RectButton btnCompress    = {165, COMP_BUTTONS_Y, 145, 50, LIGHTGREEN, "Compress"};
  RectButton btnBack        = {10,  195, 70,  40, GREY,   "Back"};
  RectButton btnGoToPrepare = {90,  195, 140, 40, ORANGE, "Prepare"};
  RectButton btnZero        = {240, 195, 70,  40, GREY,   "Zero"};
  drawButton(btnPrepare,  (motor.zeroDefined && !motor.mustClickZero));
  drawButton(btnCompress, (motor.isPrepared && motor.zeroDefined && !motor.mustClickZero));
  drawButton(btnBack); drawButton(btnGoToPrepare, motor.zeroDefined); drawButton(btnZero);
}


// =============================================================
//  SCREEN DRAWING – ADMIN PASSWORD
// =============================================================
void resetAdminPasswordInput() {
  adminPasswordInput[0] = '\0';
  adminPasswordLen = 0;
  adminPasswordError = false;
}

void drawAdminPasswordDisplay() {
  gfx->fillRect(20, 54, 280, 42, BLACK);
  drawCenteredText("Admin password", SCREEN_CENTER_X, 55, WHITE, 2);

  char masked[4];
  for (int i = 0; i < adminPasswordLen; i++) masked[i] = '*';
  masked[adminPasswordLen] = '\0';

  gfx->drawRoundRect(105, 78, 110, 28, 8, LIGHTGREY);
  drawCenteredText(masked, SCREEN_CENTER_X, 84, LIGHTGREEN, 2);

  if (adminPasswordError) {
    drawCenteredText("Wrong password", SCREEN_CENTER_X, 112, RED, 2);
  } else {
    gfx->fillRect(0, 108, TFT_WIDTH, 24, BLACK);
  }
}

void drawAdminPasswordScreen() {
  gfx->fillScreen(BLACK);
  drawCenteredText("Admin", SCREEN_CENTER_X, 16, WHITE, 2);
  drawThickHLine(40, 42, 240, UI_LINE_THICKNESS, ORANGE);
  drawAdminPasswordDisplay();

  const char* labels[10] = {"1","2","3","4","5","6","7","8","9","0"};
  for (int i = 0; i < 9; i++) {
    int col = i % 3;
    int row = i / 3;
    RectButton b = {72 + col * 60, 132 + row * 30, 48, 24, GREY, labels[i]};
    drawButton(b);
  }
  RectButton b0 = {132, 222, 48, 16, GREY, labels[9]};
  RectButton btnBack  = {8, 214, 62, 22, GREY, "Back"};
  RectButton btnClear = {250, 214, 62, 22, GREY, "Clear"};
  drawButton(b0);
  drawButton(btnClear);
  drawButton(btnBack);
}

void handleAdminPasswordMenu(int16_t x, int16_t y, bool &wasTouched) {
  if (wasTouched) return;

  RectButton btnBack  = {8, 214, 62, 22, GREY, "Back"};
  RectButton btnClear = {250, 214, 62, 22, GREY, "Clear"};

  if (hitRect(x, y, btnBack)) {
    resetAdminPasswordInput();
    ui.currentMenu = MAIN_MENU;
    drawMainScreen();
    return;
  }

  if (hitRect(x, y, btnClear)) {
    resetAdminPasswordInput();
    drawAdminPasswordScreen();
    return;
  }

  char digit = 0;
  for (int i = 0; i < 9; i++) {
    int col = i % 3;
    int row = i / 3;
    RectButton b = {72 + col * 60, 132 + row * 30, 48, 24, GREY, ""};
    if (hitRect(x, y, b)) digit = '1' + i;
  }
  RectButton b0 = {132, 222, 48, 16, GREY, ""};
  if (hitRect(x, y, b0)) digit = '0';

  if (!digit) return;

  if (adminPasswordLen < 3) {
    adminPasswordInput[adminPasswordLen++] = digit;
    adminPasswordInput[adminPasswordLen] = '\0';
  }

  if (adminPasswordLen == 3) {
    if (strcmp(adminPasswordInput, ADMIN_PASSWORD) == 0) {
      resetAdminPasswordInput();
      ui.currentMenu = ASSESSMENT_MENU;
      drawAssessmentMenu();
    } else {
      adminPasswordError = true;
      drawAdminPasswordDisplay();
      delay(500);
      resetAdminPasswordInput();
      drawAdminPasswordScreen();
    }
  } else {
    adminPasswordError = false;
    drawAdminPasswordDisplay();
  }
}

// =============================================================
//  SCREEN DRAWING – ASSESSMENT MENU
// =============================================================
#define ASSESS_BTN_W  148
#define ASSESS_BTN_H  36
#define ASSESS_COL_L  6
#define ASSESS_COL_R  166
int16_t assessRows[4] = {48, 91, 134, 177};

void getAssessButtonRect(int idx, RectButton &btn) {
  int16_t xs[2] = {ASSESS_COL_L, ASSESS_COL_R};
  btn = {xs[idx%2], assessRows[idx/2], ASSESS_BTN_W, ASSESS_BTN_H,
         testColors[idx], testNames[idx]};
}

void drawAssessmentMenu() {
  gfx->fillScreen(BLACK);
  drawCenteredText("Assessment Tests", SCREEN_CENTER_X, 10, WHITE, 2);
  drawThickHLine(10, 32, 300, UI_LINE_THICKNESS, TEAL);
  for (int i=0; i<NUM_TESTS; i++) {
    RectButton btn; getAssessButtonRect(i, btn); drawButton(btn);
  }
  RectButton btnBack = {6, 215, 80, 22, GREY, "Back"};
  drawButton(btnBack);
}

// =============================================================
//  SCREEN DRAWING – ASSESSMENT DETAIL
// =============================================================
void drawAssessmentDetail(int t) {
  gfx->fillScreen(BLACK);
  drawCenteredText(testTitles[t], SCREEN_CENTER_X, 5, GOLD, 2);
  drawThickHLine(0, 24, TFT_WIDTH, UI_LINE_THICKNESS, testColors[t]);
  gfx->setTextColor(CYAN); gfx->setTextSize(1);
  gfx->setCursor(4, 30); gfx->print("OBJECTIVE");
  printMultiLine(testObjectives[t], 4, 40, WHITE, 1, 10);
  drawThickHLine(0, 72, TFT_WIDTH, 1, DARKGREY);
  gfx->setTextColor(CYAN); gfx->setTextSize(1);
  gfx->setCursor(4, 76); gfx->print("PROCEDURE");
  printMultiLine(testProcedure[t], 4, 86, LIGHTGREY, 1, 10);
  drawThickHLine(0, 148, TFT_WIDTH, 1, DARKGREY);
  gfx->setTextColor(CYAN); gfx->setTextSize(1);
  gfx->setCursor(4, 152); gfx->print("MEASURED VARIABLES");
  printMultiLine(testVariables[t], 4, 162, LIGHTGREY, 1, 10);
  char buf[8]; snprintf(buf,sizeof(buf),"%d/%d",t+1,NUM_TESTS);
  gfx->setTextColor(DARKGREY); gfx->setTextSize(1);
  gfx->setCursor(290, 8); gfx->print(buf);
  RectButton btnBack  = {6,   215, 70,  22, GREY,          "Back"};
  RectButton btnStart = {86,  215, 140, 22, testColors[t], "START TEST"};
  drawButton(btnBack);
  drawButton(btnStart, motor.zeroDefined);
  if (t>0)         { RectButton b={232,215,40,22,DARKGREY,"<Prv"}; drawButton(b); }
  if (t<NUM_TESTS-1){ RectButton b={276,215,40,22,DARKGREY,"Nxt>"}; drawButton(b); }
  if (!motor.zeroDefined) {
    gfx->setTextColor(RED); gfx->setTextSize(1);
    gfx->setCursor(86, 207); gfx->print("Set zero first!");
  }
}

// =============================================================
//  TEST RUN SCREEN HELPERS
// =============================================================
void drawRunHeader(int t) {
  gfx->fillScreen(BLACK);
  drawCenteredText(testTitles[t], SCREEN_CENTER_X, 4, GOLD, 2);
  drawThickHLine(0, 22, TFT_WIDTH, UI_LINE_THICKNESS, testColors[t]);
}

void drawRunBottomBar(bool nextEnabled = true) {
  RectButton btnAbort = {6,   210, 80,  26, MAROON, "Abort"};
  RectButton btnNext  = {180, 210, 134, 26, GREEN,  "NEXT  >"};
  drawButton(btnAbort); drawButton(btnNext, nextEnabled);
}

void clearRunContent() { gfx->fillRect(0, 24, TFT_WIDTH, 183, BLACK); }

void showMoving() {
  clearRunContent();
  drawCenteredText("Moving...", SCREEN_CENTER_X, 50, ORANGE, 2);
}

void showMeasured(float mm, int rep, int total, const char *label) {
  clearRunContent();
  char buf[40];
  snprintf(buf,sizeof(buf),"Rep %d / %d",rep,total);
  drawCenteredText(buf, SCREEN_CENTER_X, 30, CYAN, 1);
  gfx->setTextColor(WHITE); gfx->setTextSize(2);
  gfx->setCursor(4, 52); gfx->print(label);
  snprintf(buf,sizeof(buf),"Cmd: %.4f mm",mm);
  int16_t bx,by; uint16_t bw,bh;
  gfx->getTextBounds(buf,0,0,&bx,&by,&bw,&bh);
  gfx->setTextColor(GOLD); gfx->setTextSize(2);
  gfx->setCursor(SCREEN_CENTER_X-bw/2, 82); gfx->print(buf);
  drawCenteredText("Record physical measurement externally", SCREEN_CENTER_X, 120, LIGHTGREY, 1);
  drawCenteredText("Tap  NEXT >  to continue", SCREEN_CENTER_X, 136, LIGHTGREY, 1);
}

// =============================================================
//  RESULTS SCREEN
// =============================================================
void drawResultsScreen(int t) {
  gfx->fillScreen(BLACK);
  drawCenteredText(testTitles[t], SCREEN_CENTER_X, 4, GOLD, 2);
  drawThickHLine(0, 22, TFT_WIDTH, UI_LINE_THICKNESS, testColors[t]);
  int y=30; char buf[72];

  gfx->setTextColor(CYAN); gfx->setTextSize(1);
  gfx->setCursor(4, y); gfx->print("TEST COMPLETE"); y+=16;

  auto line=[&](const char *txt, uint16_t col=WHITE){
    gfx->setTextColor(col); gfx->setTextSize(1);
    gfx->setCursor(4,y); gfx->print(txt); y+=13;
  };

  if (t==TEST_ALIGNMENT) {
    snprintf(buf,sizeof(buf),"Cycles completed: %d",tr.repsDone); line(buf);
    line("Save microscope trajectory image.", LIGHTGREY);
    line("Describe lateral deviation externally.", LIGHTGREY);

  } else if (t==TEST_POSITIONING) {
    line("Physical data to record:", CYAN);
    line("Commanded vs measured position", LIGHTGREY);
    line("for 0.05, 0.10, 0.25,", LIGHTGREY);
    line("0.50, 1.00, 1.50,", LIGHTGREY);
    line("2.00, 2.50, 3.00 mm.", LIGHTGREY);
    line("Calculate error later.", LIGHTGREY);

  } else if (t==TEST_REPEATABILITY) {
    snprintf(buf,sizeof(buf),"Repetitions completed: %d",tr.repsDone); line(buf);
    line("Record final target position", LIGHTGREY);
    line("for each repetition externally.", LIGHTGREY);
    line("Calculate std. deviation later.", LIGHTGREY);

  } else if (t==TEST_ZERO_REPEATABILITY) {
    snprintf(buf,sizeof(buf),"Repetitions completed: %d",tr.repsDone); line(buf);
    line("Record physical zero position", LIGHTGREY);
    line("after each return to zero.", LIGHTGREY);
    line("Calculate deviation later.", LIGHTGREY);

  } else if (t==TEST_BIDIRECTIONAL) {
    snprintf(buf,sizeof(buf),"Cycles completed: %d",tr.repsDone); line(buf);
    line("Record forward and reverse", LIGHTGREY);
    line("physical target positions.", LIGHTGREY);
    line("Calculate backlash later.", LIGHTGREY);

  } else if (t==TEST_DUTY_CYCLE) {
    snprintf(buf,sizeof(buf),"Cycles completed: %d",tr.repsDone); line(buf);
    line("Record temperatures at start,", LIGHTGREY);
    line("every 15 cycles, and final.", LIGHTGREY);
    line("Note performance degradation.", LIGHTGREY);

  } else if (t==TEST_CYCLIC) {
    snprintf(buf,sizeof(buf),"Full cycles completed: %d",tr.repsDone); line(buf);
    line("Record position per cycle", LIGHTGREY);
    line("and any drift/failures.", LIGHTGREY);

  } else if (t==TEST_ENDURANCE) {
    snprintf(buf,sizeof(buf),"Extended cycles completed: %d",trs.currentRep); line(buf);
    line("Run before/after physical checks:", CYAN);
    line("repeatability, backlash, zero.", LIGHTGREY);
    line("Inspect mechanical integrity.", LIGHTGREY);
  }

  drawCenteredText("Use external measurement table", SCREEN_CENTER_X, 190, GREEN, 1);
  RectButton btnBack = {6, 210, 100, 26, GREY, "< Back"};
  drawButton(btnBack);
}

// =============================================================
//  TEST EXECUTION – per-test step logic
//  Returns true when test is complete (ready for results).
// =============================================================
void initTestRun(int t) {
  memset(&tr, 0, sizeof(tr));
  tr.posTargets[0]=0.05f; tr.posTargets[1]=0.10f; tr.posTargets[2]=0.25f;
  tr.posTargets[3]=0.50f; tr.posTargets[4]=1.00f; tr.posTargets[5]=1.50f;
  tr.posTargets[6]=2.00f; tr.posTargets[7]=2.50f; tr.posTargets[8]=3.00f;
  trs = {TP_INTRO, 0, 0, 0};
  switch(t) {
    case TEST_ALIGNMENT:          tr.repsTotal=5;  break;
    case TEST_POSITIONING:        tr.repsTotal=5;  break;
    case TEST_REPEATABILITY:      tr.repsTotal=10; break;
    case TEST_ZERO_REPEATABILITY: tr.repsTotal=10; break;
    case TEST_BIDIRECTIONAL:      tr.repsTotal=10; break;
    case TEST_DUTY_CYCLE:         tr.repsTotal=300; break;
    case TEST_CYCLIC:             tr.repsTotal=15; break;
    case TEST_ENDURANCE:          tr.repsTotal=75; break;
  }
  tr.repsDone=0;
}

bool stepTestRun(int t) {

  // ===== ALIGNMENT =====
  if (t==TEST_ALIGNMENT) {
    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Alignment Test", SCREEN_CENTER_X, 40, WHITE, 2);
      gfx->setTextColor(LIGHTGREY); gfx->setTextSize(1);
      drawCenteredText("Motor sweeps full range at slow", SCREEN_CENTER_X, 70, LIGHTGREY, 1);
      drawCenteredText("speed. Observe with microscope.", SCREEN_CENTER_X, 82, LIGHTGREY, 1);
      drawCenteredText("Tap NEXT > to begin each sweep.", SCREEN_CENTER_X, 100, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; return false;
    }
    if (trs.phase==TP_AWAIT_TAP) {
      if (trs.currentRep>=tr.repsTotal) { trs.phase=TP_RESULTS; return false; }
      showMoving();
      moveToMm(TEST_DISTANCE_MAX, RPM_ALIGNMENT); delay(200);
      moveToMm(0.0f,              RPM_ALIGNMENT); delay(200);
      tr.repsDone=++trs.currentRep;
      clearRunContent();
      char buf[32]; snprintf(buf,sizeof(buf),"Cycle %d / %d done",trs.currentRep,tr.repsTotal);
      drawCenteredText(buf, SCREEN_CENTER_X, 60, WHITE, 2);
      drawCenteredText("Note any lateral deviation.", SCREEN_CENTER_X, 92, LIGHTGREY, 1);
      drawRunBottomBar(trs.currentRep < tr.repsTotal);
      if (trs.currentRep>=tr.repsTotal) {
        showMoving();
        goToZero();
        clearRunContent();
        drawCenteredText("Alignment complete", SCREEN_CENTER_X, 50, WHITE, 2);
        drawCenteredText("Returned to zero.", SCREEN_CENTER_X, 82, GREEN, 2);
        drawRunBottomBar(true);
        trs.phase=TP_RESULTS;
        return false;
      }
      return false;
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  // ===== POSITIONING ACCURACY =====
  if (t==TEST_POSITIONING) {
    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Positioning Test", SCREEN_CENTER_X, 38, WHITE, 2);
      drawCenteredText("6 targets x 5 reps each.", SCREEN_CENTER_X, 66, LIGHTGREY, 1);
      drawCenteredText("Targets: 0.5 1.0 1.5 2.0 2.5 3.0 mm", SCREEN_CENTER_X, 78, LIGHTGREY, 1);
      drawCenteredText("NEXT then returns to zero.", SCREEN_CENTER_X, 96, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; trs.subStep=0; return false;
    }
    if (trs.phase==TP_AWAIT_TAP) {
      int d=trs.targetIdx; int r=trs.currentRep;
      if (d>=NUM_POS_TARGETS) { goToZero(); return true; }

      if (trs.subStep==0) {
        showMoving();
        goToZero(); delay(150);
        moveToMm(tr.posTargets[d]);
        tr.posMeasured[d][r]=tr.posTargets[d];
        tr.repsDone++;
        char lbl[32]; snprintf(lbl,sizeof(lbl),"Target %.2f mm",tr.posTargets[d]);
        showMeasured(tr.posTargets[d], r+1, 5, lbl);
        drawCenteredText("Measure the actual position now.", SCREEN_CENTER_X, 154, LIGHTGREY, 1);
        drawCenteredText("NEXT returns to zero.", SCREEN_CENTER_X, 168, LIGHTGREY, 1);
        drawRunBottomBar(true);
        trs.subStep=1;
        return false;
      }

      if (trs.subStep==1) {
        showMoving();
        goToZero(); delay(150);
        clearRunContent();
        drawCenteredText("Returned to zero", SCREEN_CENTER_X, 52, GREEN, 2);
        drawCenteredText("Confirm zero physically if needed.", SCREEN_CENTER_X, 84, LIGHTGREY, 1);
        trs.currentRep++;
        if (trs.currentRep>=5) { trs.targetIdx++; trs.currentRep=0; }
        trs.subStep=2;
        drawRunBottomBar(true);
        return false;
      }

      if (trs.subStep==2) {
        trs.subStep=0;
        if (trs.targetIdx>=NUM_POS_TARGETS) return true;
        return stepTestRun(t);
      }
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  // ===== REPEATABILITY =====
  if (t==TEST_REPEATABILITY) {
    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Repeatability Test", SCREEN_CENTER_X, 38, WHITE, 2);
      drawCenteredText("10 reps: zero -> 2.50 mm", SCREEN_CENTER_X, 66, LIGHTGREY, 1);
      drawCenteredText("Target is held for measurement.", SCREEN_CENTER_X, 78, LIGHTGREY, 1);
      drawCenteredText("NEXT then returns to zero.", SCREEN_CENTER_X, 96, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; trs.subStep=0; return false;
    }
    if (trs.phase==TP_AWAIT_TAP) {
      if (trs.currentRep>=tr.repsTotal) { goToZero(); return true; }

      if (trs.subStep==0) {
        showMoving();
        goToZero(); delay(150);
        moveToMm(2.50f);
        tr.measured[trs.currentRep]=2.50f; tr.repsDone=trs.currentRep+1;
        showMeasured(2.50f, trs.currentRep+1, tr.repsTotal, "Target position");
        drawCenteredText("Measure the actual position now.", SCREEN_CENTER_X, 154, LIGHTGREY, 1);
        drawCenteredText("NEXT returns to zero.", SCREEN_CENTER_X, 168, LIGHTGREY, 1);
        drawRunBottomBar(true);
        trs.subStep=1;
        return false;
      }

      if (trs.subStep==1) {
        showMoving();
        goToZero(); delay(150);
        clearRunContent();
        drawCenteredText("Returned to zero", SCREEN_CENTER_X, 52, GREEN, 2);
        drawCenteredText("Tap NEXT for next repetition.", SCREEN_CENTER_X, 84, LIGHTGREY, 1);
        trs.currentRep++;
        trs.subStep=2;
        drawRunBottomBar(true);
        return false;
      }

      if (trs.subStep==2) {
        trs.subStep=0;
        if (trs.currentRep>=tr.repsTotal) return true;
        return stepTestRun(t);
      }
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  // ===== ZERO REPEATABILITY =====
  if (t==TEST_ZERO_REPEATABILITY) {
    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Zero Repeatability", SCREEN_CENTER_X, 38, WHITE, 2);
      gfx->setTextColor(LIGHTGREY); gfx->setTextSize(1);
      drawCenteredText("10 reps: move 5 mm away,", SCREEN_CENTER_X, 66, LIGHTGREY, 1);
      drawCenteredText("then return to zero.", SCREEN_CENTER_X, 78, LIGHTGREY, 1);
      drawCenteredText("Record physical zero deviation.", SCREEN_CENTER_X, 96, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; return false;
    }
    if (trs.phase==TP_AWAIT_TAP) {
      if (trs.currentRep>=tr.repsTotal) { trs.phase=TP_RESULTS; return false; }
      showMoving();
      moveToMm(5.0f); delay(100);
      moveToMm(0.0f);
      float meas=currentPosMm();
      tr.measured[trs.currentRep]=meas; tr.repsDone=trs.currentRep+1;
      showMeasured(meas, trs.currentRep+1, tr.repsTotal, "At zero");
      drawRunBottomBar(true);
      trs.currentRep++;
      if (trs.currentRep>=tr.repsTotal) trs.phase=TP_RESULTS;
      return false;
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  // ===== BIDIRECTIONAL / BACKLASH =====
  if (t==TEST_BIDIRECTIONAL) {
    const float TGT=0.50f;
    const float REVERSE_START=1.00f;
    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Bidirectional Test", SCREEN_CENTER_X, 38, WHITE, 2);
      drawCenteredText("Target: 0.50 mm. 10 cycles.", SCREEN_CENTER_X, 66, LIGHTGREY, 1);
      drawCenteredText("1: approach from zero.", SCREEN_CENTER_X, 78, LIGHTGREY, 1);
      drawCenteredText("2: approach from 1.00 mm.", SCREEN_CENTER_X, 90, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; trs.subStep=0; return false;
    }
    if (trs.phase==TP_AWAIT_TAP) {
      if (trs.currentRep>=tr.repsTotal) { goToZero(); return true; }

      if (trs.subStep==0) {
        showMoving();
        goToZero(); delay(150);
        moveToMm(TGT);
        tr.measuredFwd[trs.currentRep]=TGT;
        clearRunContent();
        char buf[44]; snprintf(buf,sizeof(buf),"Cycle %d/%d  FORWARD",trs.currentRep+1,tr.repsTotal);
        drawCenteredText(buf, SCREEN_CENTER_X, 30, CYAN, 1);
        showMeasured(TGT, trs.currentRep+1, tr.repsTotal, "Forward approach");
        drawCenteredText("Measure forward position now.", SCREEN_CENTER_X, 154, LIGHTGREY, 1);
        drawCenteredText("NEXT starts reverse approach.", SCREEN_CENTER_X, 168, LIGHTGREY, 1);
        drawRunBottomBar(true);
        trs.subStep=1;
        return false;
      }

      if (trs.subStep==1) {
        showMoving();
        moveToMm(REVERSE_START); delay(150);
        moveToMm(TGT);
        tr.measuredRev[trs.currentRep]=TGT;
        tr.repsDone=trs.currentRep+1;
        clearRunContent();
        char buf[44]; snprintf(buf,sizeof(buf),"Cycle %d/%d  REVERSE",trs.currentRep+1,tr.repsTotal);
        drawCenteredText(buf, SCREEN_CENTER_X, 30, CYAN, 1);
        showMeasured(TGT, trs.currentRep+1, tr.repsTotal, "Reverse approach");
        drawCenteredText("Measure reverse position now.", SCREEN_CENTER_X, 154, LIGHTGREY, 1);
        drawCenteredText("NEXT returns to zero.", SCREEN_CENTER_X, 168, LIGHTGREY, 1);
        drawRunBottomBar(true);
        trs.subStep=2;
        return false;
      }

      if (trs.subStep==2) {
        showMoving();
        goToZero(); delay(150);
        trs.currentRep++;
        clearRunContent();
        drawCenteredText("Returned to zero", SCREEN_CENTER_X, 52, GREEN, 2);
        drawCenteredText("Tap NEXT for next cycle.", SCREEN_CENTER_X, 84, LIGHTGREY, 1);
        trs.subStep=3;
        drawRunBottomBar(true);
        return false;
      }

      if (trs.subStep==3) {
        trs.subStep=0;
        if (trs.currentRep>=tr.repsTotal) return true;
        return stepTestRun(t);
      }
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  // ===== DUTY CYCLE =====
  if (t==TEST_DUTY_CYCLE) {
    const int DUTY_PAUSE_INTERVAL = 15;

    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Duty Cycle Test", SCREEN_CENTER_X, 38, WHITE, 2);
      drawCenteredText("300 full cycles total.", SCREEN_CENTER_X, 66, LIGHTGREY, 1);
      drawCenteredText("Pause every 15 cycles for temp.", SCREEN_CENTER_X, 78, LIGHTGREY, 1);
      drawCenteredText("Cycle: 0->Prepare->Compression->0", SCREEN_CENTER_X, 96, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; trs.subStep=0; trs.currentRep=0; return false;
    }

    if (trs.phase==TP_AWAIT_TAP) {
      if (trs.subStep==0) {
        clearRunContent();
        drawCenteredText("Record START temps now.", SCREEN_CENTER_X, 50, CYAN, 2);
        drawCenteredText("Motor and driver.", SCREEN_CENTER_X, 80, LIGHTGREY, 1);
        drawCenteredText("Tap NEXT to start cycles.", SCREEN_CENTER_X, 96, WHITE, 1);
        Serial.println(F("[DUTY CYCLE] >>> Record START temperature: motor & driver <<<"));
        drawRunBottomBar(true); trs.subStep=1; return false;
      }

      if (trs.subStep==1) {
        int blockStart = trs.currentRep;
        int blockEnd = blockStart + DUTY_PAUSE_INTERVAL;
        if (blockEnd > tr.repsTotal) blockEnd = tr.repsTotal;

        for (int c=blockStart; c<blockEnd; c++) {
          if (motor.stopFlag) break;
          clearRunContent();
          char buf[40]; snprintf(buf,sizeof(buf),"Duty cycle %d / %d",c+1,tr.repsTotal);
          drawCenteredText(buf, SCREEN_CENTER_X, 48, ORANGE, 2);
          drawCenteredText("0 -> Prepare -> Compression -> 0", SCREEN_CENTER_X, 82, LIGHTGREY, 1);
          drawCenteredText("Temp pause every 15 cycles", SCREEN_CENTER_X, 96, LIGHTGREY, 1);
          Serial.print(F("[DUTY] cycle ")); Serial.print(c+1); Serial.println(F(": 0 -> Prepare -> Compression -> 0"));
          executeAssessmentCompressionCycle(true);
          delay(150);
          trs.currentRep = c + 1;
          tr.repsDone = trs.currentRep;
        }

        goToZero();
        clearRunContent();
        if (trs.currentRep >= tr.repsTotal) {
          drawCenteredText("300 cycles complete", SCREEN_CENTER_X, 50, GREEN, 2);
          drawCenteredText("Record FINAL temps now.", SCREEN_CENTER_X, 80, CYAN, 1);
          drawCenteredText("Tap NEXT to save results.", SCREEN_CENTER_X, 96, WHITE, 1);
          Serial.println(F("[DUTY CYCLE] >>> Record FINAL temperature: motor & driver <<<"));
          drawRunBottomBar(true); trs.subStep=3; return false;
        } else {
          char buf[44]; snprintf(buf,sizeof(buf),"Pause: %d / %d cycles",trs.currentRep,tr.repsTotal);
          drawCenteredText(buf, SCREEN_CENTER_X, 44, GREEN, 2);
          drawCenteredText("Measure motor temperature.", SCREEN_CENTER_X, 76, CYAN, 1);
          drawCenteredText("Measure driver temperature.", SCREEN_CENTER_X, 90, CYAN, 1);
          drawCenteredText("Tap NEXT to continue.", SCREEN_CENTER_X, 110, WHITE, 1);
          Serial.print(F("[DUTY CYCLE] >>> Pause after ")); Serial.print(trs.currentRep);
          Serial.println(F(" cycles. Record motor & driver temperature. <<<"));
          drawRunBottomBar(true); trs.subStep=2; return false;
        }
      }

      if (trs.subStep==2) {
        trs.subStep=1;
        return stepTestRun(t);
      }

      if (trs.subStep==3) return true;
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  // ===== CYCLIC TEST =====
  if (t==TEST_CYCLIC) {
    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Cyclic Test", SCREEN_CENTER_X, 38, WHITE, 2);
      drawCenteredText("15 cycles: zero -> Prepare", SCREEN_CENTER_X, 66, LIGHTGREY, 1);
      drawCenteredText("-> Compression, STOP at compress.", SCREEN_CENTER_X, 78, LIGHTGREY, 1);
      drawCenteredText("Measure; NEXT returns to zero.", SCREEN_CENTER_X, 96, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; trs.subStep=0; return false;
    }
    if (trs.phase==TP_AWAIT_TAP) {
      if (trs.currentRep>=tr.repsTotal) { goToZero(); return true; }

      if (trs.subStep==0) {
        showMoving();
        executeAssessmentCompressionCycle(false);
        float pos=currentPosMm();
        tr.measured[trs.currentRep]=pos; tr.repsDone=trs.currentRep+1;
        showMeasured(TEST_COMPRESSION_MM, trs.currentRep+1, tr.repsTotal, "Compression position");
        drawCenteredText("Measure compression distance now.", SCREEN_CENTER_X, 154, LIGHTGREY, 1);
        drawCenteredText("NEXT returns to zero.", SCREEN_CENTER_X, 168, LIGHTGREY, 1);
        drawRunBottomBar(true);
        trs.subStep=1;
        return false;
      }

      if (trs.subStep==1) {
        showMoving();
        goToZero(); delay(150);
        trs.currentRep++;
        clearRunContent();
        drawCenteredText("Returned to zero", SCREEN_CENTER_X, 52, GREEN, 2);
        drawCenteredText("Tap NEXT for next cycle.", SCREEN_CENTER_X, 84, LIGHTGREY, 1);
        trs.subStep=2;
        drawRunBottomBar(true);
        return false;
      }

      if (trs.subStep==2) {
        trs.subStep=0;
        if (trs.currentRep>=tr.repsTotal) return true;
        return stepTestRun(t);
      }
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  // ===== ENDURANCE =====
  if (t==TEST_ENDURANCE) {
    if (trs.phase==TP_INTRO) {
      drawRunHeader(t);
      clearRunContent();
      drawCenteredText("Endurance Test", SCREEN_CENTER_X, 38, WHITE, 2);
      gfx->setTextColor(LIGHTGREY); gfx->setTextSize(1);
      drawCenteredText("75 continuous cycles:", SCREEN_CENTER_X, 66, LIGHTGREY, 1);
      drawCenteredText("0 -> Prepare -> Compression -> 0.", SCREEN_CENTER_X, 78, LIGHTGREY, 1);
      drawCenteredText("Tap NEXT to run all cycles.", SCREEN_CENTER_X, 96, LIGHTGREY, 1);
      drawRunBottomBar(true);
      trs.phase=TP_AWAIT_TAP; return false;
    }
    if (trs.phase==TP_AWAIT_TAP) {
      if (trs.currentRep>=tr.repsTotal) { trs.phase=TP_RESULTS; return false; }
      for (int c=trs.currentRep; c<tr.repsTotal; c++) {
        if (motor.stopFlag) break;
        clearRunContent();
        char buf[32]; snprintf(buf,sizeof(buf),"Cycle %d / %d",c+1,tr.repsTotal);
        drawCenteredText(buf, SCREEN_CENTER_X, 50, ORANGE, 2);
        drawCenteredText("0 -> Prepare -> Compression -> 0", SCREEN_CENTER_X, 82, LIGHTGREY, 1);
        Serial.print(F("[ENDURANCE] cycle ")); Serial.print(c+1); Serial.println(F(": 0 -> Prepare -> Compression -> 0"));
        executeAssessmentCompressionCycle(true);
        if (c<MAX_REPS) { tr.measured[c]=TEST_COMPRESSION_MM; }
        tr.repsDone=c+1;
        trs.currentRep=c+1;
        delay(100);
      }
      goToZero();
      clearRunContent();
      char buf[44]; snprintf(buf,sizeof(buf),"Done %d / %d cycles",trs.currentRep,tr.repsTotal);
      drawCenteredText(buf, SCREEN_CENTER_X, 55, WHITE, 2);
      drawCenteredText("Record checks externally.", SCREEN_CENTER_X, 86, GOLD, 1);
      drawRunBottomBar(true);
      trs.phase=TP_RESULTS;
      return false;
    }
    if (trs.phase==TP_RESULTS) return true;
  }

  return false;
}

// =============================================================
//  TOUCH MAPPING
// =============================================================
void mapTouchToScreen(int16_t &x, int16_t &y) {
  TS_Point p=ts.getPoint();
  int16_t tx=map(p.x,TS_MINX,TS_MAXX,0,TFT_WIDTH);
  int16_t ty=map(p.y,TS_MINY,TS_MAXY,0,TFT_HEIGHT);
  x=TFT_WIDTH-1-tx; y=TFT_HEIGHT-1-ty;
}

// =============================================================
//  TOUCH HANDLERS – EXISTING MENUS
// =============================================================
void handleCalibrationMenu(int16_t x, int16_t y, bool &wasTouched) {
  RectButton btnBwd  = {10,  80, 95,  40, GREY,   "Bwd"};
  RectButton btnFwd  = {110, 80, 95,  40, GREY,   "Fwd"};
  RectButton btnDef  = {210, 80, 100, 40, GREY,   "D.Zero"};
  RectButton btnTest = {10, 130, 300, 50, ORANGE, "Test"};
  RectButton btnBack = {10, 195, 70,  40, GREY,   "Back"};
  RectButton btnZero = {240,195, 70,  40, GREY,   "Zero"};
  bool fwd=hitRect(x,y,btnFwd), bwd=hitRect(x,y,btnBwd);
  if (fwd||bwd) {
    if (!jog.active||jog.direction!=fwd) {
      jog.direction=fwd; digitalWrite(PIN_DIR,fwd?HIGH:LOW);
      enableMotor(true); jog.lastStepTime=micros(); jog.active=true;
    }
  } else {
    if (jog.active){enableMotor(false);jog.active=false;}
    if (!wasTouched) {
      if      (hitRect(x,y,btnDef))  { defineZero(); updateZeroStatusText(); }
      else if (hitRect(x,y,btnTest)) { testCalibration(); }
      else if (hitRect(x,y,btnBack)) { ui.currentMenu=MAIN_MENU; jog.active=false; drawMainScreen(); }
      else if (hitRect(x,y,btnZero)&&motor.zeroDefined) { goToZero(); }
    }
  }
}

void handleCompressionMenu(int16_t x, int16_t y, bool &wasTouched) {
  RectButton btnPrepare     = {10,  COMP_BUTTONS_Y, 145, 50, ORANGE,     "Prepare"};
  RectButton btnCompress    = {165, COMP_BUTTONS_Y, 145, 50, LIGHTGREEN, "Compress"};
  RectButton btnBack        = {10,  195, 70,  40, GREY,   "Back"};
  RectButton btnGoToPrepare = {90,  195, 140, 40, ORANGE, "Prepare"};
  RectButton btnZero        = {240, 195, 70,  40, GREY,   "Zero"};
  if (!wasTouched) {
    if (hitRect(x,y,btnPrepare)&&motor.zeroDefined&&!motor.mustClickZero) {
      prepareDistancemm();
      drawButton(btnCompress,(motor.isPrepared&&motor.zeroDefined&&!motor.mustClickZero));
      drawButton(btnGoToPrepare,motor.zeroDefined);
    } else if (hitRect(x,y,btnCompress)&&motor.zeroDefined&&motor.isPrepared&&!motor.mustClickZero) {
      compressDistancemm();
    } else if (hitRect(x,y,btnGoToPrepare)&&motor.zeroDefined) {
      if (isAtPreparePosition()) {
        updateStatusText(COMP_STATUS_Y,"Already at position",GREEN);
      } else {
        goToPreparePosition();
        updateStatusText(COMP_STATUS_Y,"At prepare position",GREEN);
      }
      ui.statusMessageStartTime=millis();
    } else if (hitRect(x,y,btnZero)&&motor.zeroDefined) {
      goToZero();
      if (motor.mustClickZero) {
        motor.mustClickZero=false; motor.isPrepared=false;
        drawButton(btnPrepare,(motor.zeroDefined&&!motor.mustClickZero));
        drawButton(btnCompress,(motor.isPrepared&&motor.zeroDefined&&!motor.mustClickZero));
        drawButton(btnGoToPrepare,motor.zeroDefined);
        updateCompressionDisplayStatus();
      }
    } else if (hitRect(x,y,btnBack)) { ui.currentMenu=MAIN_MENU; drawMainScreen(); }
  }
}

// =============================================================
//  TOUCH HANDLERS – ASSESSMENT
// =============================================================
void handleAssessmentMenu(int16_t x, int16_t y, bool &wasTouched) {
  if (wasTouched) return;
  for (int i=0;i<NUM_TESTS;i++) {
    RectButton btn; getAssessButtonRect(i,btn);
    if (hitRect(x,y,btn)) {
      ui.selectedTest=i; ui.currentMenu=ASSESSMENT_DETAIL;
      drawAssessmentDetail(i); return;
    }
  }
  RectButton btnBack={6,215,80,22,GREY,"Back"};
  if (hitRect(x,y,btnBack)) { ui.currentMenu=MAIN_MENU; drawMainScreen(); }
}

void handleAssessmentDetail(int16_t x, int16_t y, bool &wasTouched) {
  if (wasTouched) return;
  int t=ui.selectedTest;
  RectButton btnBack  = {6,   215, 70,  22, GREY,          "Back"};
  RectButton btnStart = {86,  215, 140, 22, testColors[t], "START TEST"};
  RectButton btnPrev  = {232, 215, 40,  22, DARKGREY,      "<Prv"};
  RectButton btnNext  = {276, 215, 40,  22, DARKGREY,      "Nxt>"};
  if (hitRect(x,y,btnBack)) {
    ui.currentMenu=ASSESSMENT_MENU; drawAssessmentMenu();
  } else if (hitRect(x,y,btnStart)&&motor.zeroDefined) {
    initTestRun(t);
    ui.currentMenu=ASSESSMENT_RUN;
    bool done=stepTestRun(t);
    if (done) { serialPrintResults(); drawResultsScreen(t); trs.phase=TP_RESULTS; }
  } else if (t>0&&hitRect(x,y,btnPrev)) {
    ui.selectedTest--; drawAssessmentDetail(ui.selectedTest);
  } else if (t<NUM_TESTS-1&&hitRect(x,y,btnNext)) {
    ui.selectedTest++; drawAssessmentDetail(ui.selectedTest);
  }
}

void handleAssessmentRun(int16_t x, int16_t y, bool &wasTouched) {
  if (wasTouched) return;
  int t=ui.selectedTest;
  RectButton btnAbort = {6,   210, 80,  26, MAROON, "Abort"};
  RectButton btnNext  = {180, 210, 134, 26, GREEN,  "NEXT  >"};
  RectButton btnBack  = {6,   210, 100, 26, GREY,   "< Back"};

  if (trs.phase==TP_RESULTS) {
    if (hitRect(x,y,btnBack)) {
      trs.phase=TP_IDLE; ui.currentMenu=ASSESSMENT_MENU; drawAssessmentMenu();
    }
    return;
  }
  if (hitRect(x,y,btnAbort)) {
    motor.stopFlag=true; delay(50); goToZero();
    trs.phase=TP_IDLE; ui.currentMenu=ASSESSMENT_DETAIL; drawAssessmentDetail(t);
    return;
  }
  if (hitRect(x,y,btnNext)) {
    bool done=stepTestRun(t);
    if (done) { serialPrintResults(); drawResultsScreen(t); trs.phase=TP_RESULTS; }
  }
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200); delay(1000);
  pinMode(PIN_STEP,OUTPUT); pinMode(PIN_DIR,OUTPUT); pinMode(PIN_EN,OUTPUT);
  digitalWrite(PIN_STEP,LOW); enableMotor(false);
  pinMode(TFT_CS,OUTPUT); pinMode(TOUCH_CS,OUTPUT);
  digitalWrite(TFT_CS,HIGH); digitalWrite(TOUCH_CS,HIGH);
  gfx->begin(); gfx->setRotation(3);
  ts.begin(); ts.setRotation(3);
  randomSeed(analogRead(A0));
  jog.periodUs=rpmToPeriodUs(RPM_NORMAL);
  ui.currentMenu=MAIN_MENU; drawMainScreen();
}

// =============================================================
//  LOOP
// =============================================================
void loop() {
  static bool wasTouched=false;
  unsigned long now=millis();

  if (ui.showZeroMessage&&(now-ui.messageStartTime)>=MESSAGE_DURATION) {
    ui.showZeroMessage=false;
    if (ui.currentMenu==CALIBRATION_MENU) updateZeroStatusText();
  }
  if (!ui.isTesting&&ui.currentMenu==CALIBRATION_MENU&&
      ui.statusMessageStartTime!=0&&(now-ui.statusMessageStartTime)>=STATUS_MESSAGE_DURATION) {
    ui.statusMessageStartTime=0; updateZeroStatusText();
  }
  if (!ui.isPreparing&&!ui.isCompressing&&ui.currentMenu==COMPRESSION_MENU&&
      ui.statusMessageStartTime!=0&&(now-ui.statusMessageStartTime)>=STATUS_MESSAGE_DURATION) {
    ui.statusMessageStartTime=0; updateCompressionDisplayStatus();
  }
  if (ui.pendingPostCompress&&ui.currentMenu==COMPRESSION_MENU&&
      (now-ui.finishedMessageTime)>=FINISHED_DISPLAY_DURATION) {
    ui.pendingPostCompress=false; motor.mustClickZero=true; motor.isPrepared=false;
    RectButton btnPrepare     = {10,  COMP_BUTTONS_Y, 145, 50, ORANGE,     "Prepare"};
    RectButton btnCompress    = {165, COMP_BUTTONS_Y, 145, 50, LIGHTGREEN, "Compress"};
    RectButton btnGoToPrepare = {90,  195, 140, 40, ORANGE, "Prepare"};
    drawButton(btnPrepare,(motor.zeroDefined&&!motor.mustClickZero));
    drawButton(btnCompress,(motor.isPrepared&&motor.zeroDefined&&!motor.mustClickZero));
    drawButton(btnGoToPrepare,motor.zeroDefined);
    updateCompressionDisplayStatus();
  }
  if (ui.currentMenu==CALIBRATION_MENU&&!ui.isTesting) {
    int cur=motor.zeroDefined?1:0;
    if (cur!=ui.previousZeroState&&!ui.showZeroMessage) {
      ui.previousZeroState=cur; updateZeroStatusText();
    }
  }
  if (ui.currentMenu==COMPRESSION_MENU) {
    int cur=motor.zeroDefined?1:0;
    if (cur!=ui.previousCompressionZeroState) {
      ui.previousCompressionZeroState=cur;
      if (!motor.zeroDefined){motor.isPrepared=false;motor.mustClickZero=false;}
      updateCompressionDisplayStatus();
      RectButton btnPrepare     = {10,  COMP_BUTTONS_Y, 145, 50, ORANGE,     "Prepare"};
      RectButton btnCompress    = {165, COMP_BUTTONS_Y, 145, 50, LIGHTGREEN, "Compress"};
      RectButton btnGoToPrepare = {90,  195, 140, 40, ORANGE, "Prepare"};
      bool pe=(motor.zeroDefined&&!motor.mustClickZero);
      bool ce=(motor.isPrepared&&motor.zeroDefined&&!motor.mustClickZero);
      drawButton(btnPrepare,pe); drawButton(btnCompress,ce);
      drawButton(btnGoToPrepare,motor.zeroDefined);
      ui.previousPrepareEnabled=pe; ui.previousCompressEnabled=ce;
    }
    bool pe=(motor.zeroDefined&&!motor.mustClickZero);
    if (pe!=ui.previousPrepareEnabled) {
      ui.previousPrepareEnabled=pe;
      RectButton b={10,COMP_BUTTONS_Y,145,50,ORANGE,"Prepare"}; drawButton(b,pe);
    }
    bool ce=(motor.isPrepared&&motor.zeroDefined&&!motor.mustClickZero);
    if (ce!=ui.previousCompressEnabled) {
      ui.previousCompressEnabled=ce;
      RectButton b={165,COMP_BUTTONS_Y,145,50,LIGHTGREEN,"Compress"}; drawButton(b,ce);
    }
  }

  bool nowTouched=ts.touched();
  if (nowTouched) {
    int16_t x,y; mapTouchToScreen(x,y);
    if (x<0||x>=TFT_WIDTH||y<0||y>=TFT_HEIGHT){wasTouched=nowTouched;return;}

    if (ui.currentMenu==MAIN_MENU&&!wasTouched) {
      RectButton btnCalib = {25,105,270,42,GREY,"Calibration"};
      RectButton btnComp  = {25,158,270,42,GREY,"Compression"};
      RectButton btnAdmin = {244,214,70,20,GREY,"Admin"};
      if (hitRect(x,y,btnCalib)) { ui.currentMenu=CALIBRATION_MENU; drawCalibrationScreen(); }
      else if (hitRect(x,y,btnComp)) {
        ui.currentMenu=COMPRESSION_MENU;
        if (!ui.compressionMenuEntered){motor.isPrepared=false;motor.mustClickZero=false;ui.compressionMenuEntered=true;}
        drawCompressionScreen();
      } else if (hitRect(x,y,btnAdmin)) {
        resetAdminPasswordInput();
        ui.currentMenu=ADMIN_PASSWORD_MENU;
        drawAdminPasswordScreen();
      }

    } else if (ui.currentMenu==CALIBRATION_MENU&&!ui.showZeroMessage&&!ui.isTesting) {
      handleCalibrationMenu(x,y,wasTouched);
    } else if (ui.currentMenu==COMPRESSION_MENU&&!ui.isPreparing&&!ui.isCompressing&&!ui.pendingPostCompress) {
      handleCompressionMenu(x,y,wasTouched);
    } else if (ui.currentMenu==ADMIN_PASSWORD_MENU) { handleAdminPasswordMenu(x,y,wasTouched); }
    else if (ui.currentMenu==ASSESSMENT_MENU)  { handleAssessmentMenu(x,y,wasTouched);  }
    else if (ui.currentMenu==ASSESSMENT_DETAIL)  { handleAssessmentDetail(x,y,wasTouched); }
    else if (ui.currentMenu==ASSESSMENT_RUN)     { handleAssessmentRun(x,y,wasTouched);    }

  } else {
    if (jog.active){enableMotor(false);jog.active=false;}
  }

  if (jog.active) {
    unsigned long nowUs=micros();
    if ((unsigned long)(nowUs-jog.lastStepTime)>=jog.periodUs) {
      stepMotor(jog.direction);
      if (jog.direction) motor.currentPosSteps++; else motor.currentPosSteps--;
      jog.lastStepTime=nowUs;
    }
  }
  wasTouched=nowTouched;
}
