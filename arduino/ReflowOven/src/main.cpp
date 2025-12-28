#include <Arduino.h>

// ---------------------------
// Hardware config (EDIT THESE)
// ---------------------------
static const int SSR_PIN = 7;   // SSR control pin (digital output)

// ---------------------------
// Profile definition
// ---------------------------
enum Phase : uint8_t { PREHEAT = 0, SOAK = 1, REFLOW = 2, COOL = 3, PHASE_COUNT = 4 };

struct Step
{
  const char* label;
  float duration_s;
  float startC;
  float endC;
};

static const Step profile[PHASE_COUNT] =
{
  { "PREHEAT", 150.0f,  25.0f, 150.0f },
  { "SOAK",    120.0f, 150.0f, 180.0f },
  { "REFLOW",   45.0f, 180.0f, 225.0f },
  { "COOL",    120.0f, 225.0f,  50.0f }
};

// ---------------------------
// Adaptive profile follow tuning
// ---------------------------
static const float TIME_WARP_GAIN = 0.035f; // 1/°C : slow down when behind, speed up when ahead
static const float MIN_TIME_RATE  = 0.15f;
static const float MAX_TIME_RATE  = 1.50f;

static const float MAX_RAMP_UP_CPS   = 1.00f; // °C/s ramp limiting for setpoint
static const float MAX_RAMP_DOWN_CPS = 2.00f;

// ---------------------------
// Simple PID (you can replace with your existing PID)
// ---------------------------
static float Kp = 18.0f;
static float Ki = 0.08f;
static float Kd = 120.0f;

static float pidIntegral = 0.0f;
static float lastTemp = 0.0f;
static bool pidHasLast = false;

// Output is 0..100 (% duty)
static float pidCompute(float setpoint, float temp, float dt)
{
  float err = setpoint - temp;

  pidIntegral += err * dt;
  // basic anti-windup clamp
  if (pidIntegral > 500.0f) pidIntegral = 500.0f;
  if (pidIntegral < -500.0f) pidIntegral = -500.0f;

  float deriv = 0.0f;
  if (pidHasLast && dt > 0.0f)
    deriv = (temp - lastTemp) / dt; // dTemp/dt

  lastTemp = temp;
  pidHasLast = true;

  // Note: derivative on measurement (temp), so subtract sign
  float out = Kp * err + Ki * pidIntegral - Kd * deriv;

  if (out < 0.0f) out = 0.0f;
  if (out > 100.0f) out = 100.0f;
  return out;
}

// ---------------------------
// SSR time-proportioning window
// ---------------------------
static const uint32_t SSR_WINDOW_MS = 1000;
static uint32_t windowStartMs = 0;

static void ssrWritePercent(float percent)
{
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  uint32_t now = millis();
  if (now - windowStartMs >= SSR_WINDOW_MS)
    windowStartMs = now;

  uint32_t onTime = (uint32_t)(SSR_WINDOW_MS * (percent / 100.0f));
  digitalWrite(SSR_PIN, (now - windowStartMs) < onTime ? HIGH : LOW);
}

// ---------------------------
// State machine
// ---------------------------
enum RunState : uint8_t { IDLE = 0, RUNNING = 1, DONE = 2, ABORTED = 3 };

static RunState runState = IDLE;
static Phase phase = PREHEAT;

static float currentTempC = 25.0f;
static float setpointC = 25.0f;

// Adaptive timebase
static float profileTime_s = 0.0f;          // PT: seconds (warped time)
static float phaseStartPT_s = 0.0f;         // PT at phase start
static float lastLoopTime_s = 0.0f;

// Telemetry pacing
static uint32_t lastTelemMs = 0;
static const uint32_t TELEMETRY_PERIOD_MS = 200;

// ---------------------------
// Temperature input (REPLACE THIS with your sensor code)
// ---------------------------
static float readTemperatureC()
{
  // TODO: Replace with your thermocouple/RTD reading.
  // If you already have code, paste it here and return the measured °C.
  //
  // Example placeholders:
  // return thermocouple.readCelsius();
  // return analogRead(A0) * scale + offset;
  //
  return currentTempC; // placeholder (will not work unless replaced)
}

// ---------------------------
// Helpers
// ---------------------------
static const char* phaseName(Phase p)
{
  return profile[p].label;
}

static const char* stateName(RunState s)
{
  switch (s)
  {
    case IDLE: return "IDLE";
    case RUNNING: return "RUNNING";
    case DONE: return "DONE";
    case ABORTED: return "ABORTED";
    default: return "IDLE";
  }
}

static void startRun()
{
  runState = RUNNING;
  phase = PREHEAT;

  pidIntegral = 0.0f;
  pidHasLast = false;

  profileTime_s = 0.0f;
  phaseStartPT_s = 0.0f;

  lastLoopTime_s = millis() * 0.001f;

  setpointC = profile[PREHEAT].startC;

  windowStartMs = millis();
}

static void abortRun()
{
  runState = ABORTED;
  digitalWrite(SSR_PIN, LOW);
}

static void nextPhase()
{
  if (phase < (Phase)(PHASE_COUNT - 1))
  {
    phase = (Phase)((uint8_t)phase + 1);
    phaseStartPT_s = profileTime_s;

    // reset some PID memory between phases if desired
    // pidIntegral = 0.0f;
    // pidHasLast = false;

    // keep setpoint continuous (no step)
  }
  else
  {
    runState = DONE;
    digitalWrite(SSR_PIN, LOW);
  }
}

// ---------------------------
// Serial command handling
// ---------------------------
static void handleSerial()
{
  while (Serial.available() > 0)
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      continue;

    line.toUpperCase();

    if (line == "START")
      startRun();
    else if (line == "ABORT")
      abortRun();
  }
}

// ---------------------------
// Main control update
// ---------------------------
static void updateControl()
{
  float now_s = millis() * 0.001f;
  float dt = now_s - lastLoopTime_s;
  if (dt <= 0.0f || dt > 2.0f)
  {
    lastLoopTime_s = now_s;
    return;
  }
  lastLoopTime_s = now_s;

  currentTempC = readTemperatureC();

  if (runState != RUNNING)
  {
    ssrWritePercent(0.0f);
    return;
  }

  // Compute current phase parameters
  const Step& st = profile[phase];
  float phaseDuration = st.duration_s;
  float phaseStart = st.startC;
  float phaseEnd = st.endC;

  // -------- Adaptive time-warp --------
  float err = setpointC - currentTempC;
  float timeRate = 1.0f - TIME_WARP_GAIN * err;
  if (timeRate < MIN_TIME_RATE) timeRate = MIN_TIME_RATE;
  if (timeRate > MAX_TIME_RATE) timeRate = MAX_TIME_RATE;

  // Advance profile time
  profileTime_s += dt * timeRate;

  // Phase-local time
  float phaseTime = profileTime_s - phaseStartPT_s;

  // Phase complete?
  if (phaseTime >= phaseDuration)
  {
    nextPhase();
    return;
  }

  // Compute ideal target setpoint for this phase from profileTime
  float progress = phaseTime / phaseDuration;
  if (progress < 0.0f) progress = 0.0f;
  if (progress > 1.0f) progress = 1.0f;

  float targetSetpoint = phaseStart + (phaseEnd - phaseStart) * progress;

  // -------- Ramp limit setpoint --------
  float maxUp = MAX_RAMP_UP_CPS * dt;
  float maxDown = MAX_RAMP_DOWN_CPS * dt;

  if (targetSetpoint > setpointC)
    setpointC = min(targetSetpoint, setpointC + maxUp);
  else
    setpointC = max(targetSetpoint, setpointC - maxDown);

  // -------- Optional PREHEAT assist --------
  // If you want to blast full power at the beginning to reduce dead-time:
  if (phase == PREHEAT && currentTempC < phaseStart + 10.0f)
  {
    ssrWritePercent(100.0f);
    return;
  }

  // COOL behavior: heater off (your UI already prompts to open the door)
  if (phase == COOL)
  {
    ssrWritePercent(0.0f);
    return;
  }

  // PID -> SSR duty
  float duty = pidCompute(setpointC, currentTempC, dt);
  ssrWritePercent(duty);
}

// ---------------------------
// Telemetry output
// ---------------------------
static void sendTelemetry()
{
  uint32_t now = millis();
  if (now - lastTelemMs < TELEMETRY_PERIOD_MS)
    return;
  lastTelemMs = now;

  Serial.print("T:");
  Serial.print(currentTempC, 2);
  Serial.print(";S:");
  Serial.print(setpointC, 2);
  Serial.print(";STATE:");
  Serial.print(stateName(runState));
  Serial.print(";PHASE:");
  Serial.print(phaseName(phase));
  Serial.print(";PT:");
  Serial.print(profileTime_s, 1);
  Serial.println();
}

void setup()
{
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  runState = IDLE;
  phase = PREHEAT;
  currentTempC = 25.0f;
  setpointC = 25.0f;

  windowStartMs = millis();
  lastLoopTime_s = millis() * 0.001f;
}

void loop()
{
  handleSerial();

  // If you want ABORTED to return to IDLE when user hits START again, that's already handled.
  updateControl();
  sendTelemetry();
}
