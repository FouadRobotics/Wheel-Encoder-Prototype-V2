// ================= CONFIG =================
const int sensor1Pin = A0;
const int sensor2Pin = A1;

// Offsets (calibrated)
const int offset1 = 507;
const int offset2 = 499;

// Per-channel hysteresis thresholds (tune these!)
const int HIGH_TH_1 = 25;
const int LOW_TH_1  = -25;

const int HIGH_TH_2 = 25;
const int LOW_TH_2  = -25;

// Phase alignment (delay for channel 2)
#define DELAY_LEN 3   // adjust 0–5 to tune phase
int delayBuffer[DELAY_LEN] = {0};
int delayIdx = 0;

// ================= STATE =================
int state1 = 0;
int state2 = 0;

int prevState = 0;
long encoderCount = 0;

// Timing (non-blocking)
unsigned long lastMicros = 0;
const int SAMPLE_PERIOD_US = 4000;  // 2 kHz sampling

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
}

// ================= LOOP =================
void loop() {
  if (micros() - lastMicros < SAMPLE_PERIOD_US) return;
  lastMicros = micros();

  // ---- Read sensors ----
  int val1 = analogRead(sensor1Pin) - offset1;
  int val2 = analogRead(sensor2Pin) - offset2;

  // ---- Hysteresis channel 1 ----
  if (val1 > HIGH_TH_1) {
    state1 = 1;
  } else if (val1 < LOW_TH_1) {
    state1 = 0;
  }

  // ---- Hysteresis channel 2 ----
  if (val2 > HIGH_TH_2) {
    state2 = 1;
  } else if (val2 < LOW_TH_2) {
    state2 = 0;
  }

  // ---- Phase alignment (delay channel 2) ----
  delayBuffer[delayIdx] = state2;
  delayIdx = (delayIdx + 1) % DELAY_LEN;
  int alignedState2 = delayBuffer[delayIdx];

  // ---- Combine into 2-bit state ----
  int currentState = (state1 << 1) | alignedState2;

  // ---- Quadrature decoding (state machine) ----
  int transition = (prevState << 2) | currentState;

  switch (transition) {
    // Forward
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCount++;
      break;

    // Reverse
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCount--;
      break;

    default:
      // Invalid transition → ignore
      currentState = prevState;
      break;
  }

  prevState = currentState;

  // ---- Output ----
  Serial.print(state1);
  Serial.print(",");
  Serial.print(alignedState2);
  Serial.print(",");
  Serial.println(encoderCount);
}