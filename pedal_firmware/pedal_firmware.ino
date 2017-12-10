#include "MIDIUSB.h"



// === CONSTANTS ===
const int NUM_BANKS = 4;
const byte MIDI_CHANNEL = 7;

const float FILTER_MAX_RATE = 4.2;  // Max amount expr or knob can change in a second
const int POT_LOWER = 132;  // deadband of 4 added to both of these
const int POT_UPPER = 1019;
// NOTE Expr pedal seems to follow curve of format y = A*B^(-Dx) + C
// These values were found by plugging in the max and min values of the
// pedal, then solving for all values while tweaking B and D.
// Padded raw max and min measured at [80, 250]
// https://www.desmos.com/calculator/cv4xtfvs05
const float EXPR_CURVE_A = 287.8;
const float EXPR_CURVE_B = 1.2;
const float EXPR_CURVE_C = -37.8;
const float EXPR_CURVE_D = 4.9;
const float EXPR_CURVE_DENOM = log(EXPR_CURVE_B) * EXPR_CURVE_D;

const byte FS_BANK_CCS[NUM_BANKS][2] = {
  {20, 21},
  {22, 23},
  {24, 25},
  {67, 64}  // These cc's are for piano soft and sustain pedals
};
// Foot controller, mod wheel, and two undefineds
const byte EXPR_BANK_CCS[NUM_BANKS] = {4, 1, 14, 15};
const byte KNOB_BANK_CCS[NUM_BANKS] = {102, 103, 104, 105};

const int PRESS_EVENT = 1;
const int RELEASE_EVENT = -1;
const int NO_EVENT = 0;



// === PINS ===
int toggleLedPins[] = {15, 16};
int statusLedPins[] = {14, 10};

int exprPin = A0;
int fsBankPin = A3;
int knobPin = A2;
int exprBankPin = A1;

int fsPins[] = {3, 2};
int toggleButtonPins[] = {5, 4};
int extraButtonPin = 6;



// === STATE ===
int lastMillis = 0;
float dt = 0;
int currentFsBank = 0;
int currentExprBank = 0;

bool toggleStates[NUM_BANKS][2];  // Whether toggle mode is on or off for each fs in each bank
bool fsMidi[NUM_BANKS][2];  // The last state of each fs control sent over midi
bool physFsState[2];  // The last physical pressed state of each fs
bool physToggleState[2];  // The last physical pressed state of each toggle

float lastExpr = 0;
float lastKnob = 0;



// === HELPERS ===
float normalize(int value, int lower, int upper) {
  value -= lower;
  return clamp(value / ((float) (upper - lower)));
}

float clamp(float val) {
  return min(1, max(0, val));
}

void maybeResetFs(int i) {
  if(!toggleStates[currentFsBank][i]) {
    writeFs(i, false);
  }
}

void writeFs(int i, bool val) {
  if(fsMidi[currentFsBank][i] != val) {
    writeCc(FS_BANK_CCS[currentFsBank][i], val ? 127 : 0);
    fsMidi[currentFsBank][i] = val;
  }
}

void writeCc(byte cc, byte val) {
  midiEventPacket_t event = {0x0B, 0xB0 | MIDI_CHANNEL, cc, val};
  MidiUSB.sendMIDI(event);
}

int getPressEvent(int pin, bool* state) {
  int event = NO_EVENT;
  bool pressed = (digitalRead(pin) == LOW);

  if(pressed && !*state) {
    event = PRESS_EVENT;
  }
  else if(!pressed && *state) {
    event = RELEASE_EVENT;
  }

  *state = pressed;
  return event;
}

void updateFs(int i) {
  int event = getPressEvent(fsPins[i], &physFsState[i]);
  bool isToggle = toggleStates[currentFsBank][i];

  if(event == PRESS_EVENT && isToggle) {
    writeFs(i, !fsMidi[currentFsBank][i]);
  }
  else if(event == PRESS_EVENT) {
    writeFs(i, true);
  }
  else if(event == RELEASE_EVENT && !isToggle) {
    writeFs(i, false);
  }
}

void updateKnob() {
  float normalized = normalize(analogRead(knobPin), POT_LOWER, POT_UPPER);
  updateControl(normalized, &lastKnob, KNOB_BANK_CCS[currentExprBank]);
}

void updateExpr() {
  int raw = analogRead(exprPin);
  Serial.println(raw);
  float logArg = max(0, (raw - EXPR_CURVE_C) / EXPR_CURVE_A);
  float unboundedOut = -log(logArg) / EXPR_CURVE_DENOM;
  updateControl(clamp(unboundedOut), &lastExpr, EXPR_BANK_CCS[currentExprBank]);
}

void updateControl(float normalized, float* state, int cc) {
  float direction = normalized > *state ? 1 : -1;
  float change = abs(normalized - *state);
  float rate = change / dt;

  if(rate > FILTER_MAX_RATE) {
    change = FILTER_MAX_RATE * dt;
  }
  float filtered = *state + (change * direction);

  byte prevMidi = min(127, floor(*state * 128));
  byte desiredMidi = min(127, floor(filtered * 128));

  if(desiredMidi != prevMidi) {
    writeCc(cc, desiredMidi);
  }

  *state = filtered;
}

void handleBankChanges() {
  float fsBankReading = normalize(
    analogRead(fsBankPin), POT_LOWER, POT_UPPER);
  float exprBankReading = normalize(
    analogRead(exprBankPin), POT_LOWER, POT_UPPER);
  int desiredFsBank = min(NUM_BANKS - 1, floor(fsBankReading * NUM_BANKS));
  int desiredExprBank = min(NUM_BANKS - 1, floor(exprBankReading * NUM_BANKS));

  if(desiredFsBank != currentFsBank) {
    maybeResetFs(0);
    maybeResetFs(1);
  }

  currentFsBank = desiredFsBank;
  currentExprBank = desiredExprBank;
}

void updateToggleState(int i) {
  if(getPressEvent(toggleButtonPins[i], &physToggleState[i]) == PRESS_EVENT) {
    toggleStates[currentFsBank][i] = !toggleStates[currentFsBank][i];
    maybeResetFs(0);
    maybeResetFs(1);
  }
}

void sendControls() {
  updateFs(0);
  updateFs(1);
  // NOTE - Expression pedal too noisy. Disabled.
  // updateExpr();
  updateKnob();

  MidiUSB.flush();
}

void updateFsDisplay(int i) {
  digitalWrite(
    toggleLedPins[i],
    toggleStates[currentFsBank][i] ? HIGH : LOW);
  digitalWrite(
    statusLedPins[i],
    fsMidi[currentFsBank][i] ? HIGH : LOW);
}



// === MAIN STUFF ===
void setup() {
  // Outputs
  pinMode(toggleLedPins[0], OUTPUT);
  pinMode(toggleLedPins[1], OUTPUT);
  pinMode(statusLedPins[0], OUTPUT);
  pinMode(statusLedPins[1], OUTPUT);
  Serial.begin(115200);

  // Analog Inputs
  pinMode(exprPin, INPUT);
  pinMode(fsBankPin, INPUT);
  pinMode(knobPin, INPUT);
  pinMode(exprBankPin, INPUT);

  // Digital Inputs
  pinMode(fsPins[0], INPUT);
  pinMode(fsPins[1], INPUT);
  pinMode(toggleButtonPins[0], INPUT);
  pinMode(toggleButtonPins[1], INPUT);

  pinMode(extraButtonPin, INPUT);

  // Blink green light to indicate end of startup
  digitalWrite(toggleLedPins[1], HIGH);
  delay(1000);
  digitalWrite(toggleLedPins[1], LOW);
}


void loop() {
  int currentMillis = millis();
  dt = (currentMillis - lastMillis) / 1000.0;
  lastMillis = currentMillis;

  handleBankChanges();

  updateToggleState(0);
  updateToggleState(1);
  updateFsDisplay(0);
  updateFsDisplay(1);

  sendControls();
}
