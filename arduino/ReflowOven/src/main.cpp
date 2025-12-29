#include <Arduino.h>
#include <math.h>

// -------------------- Pins --------------------
const int THERMISTOR_PIN = A0;
const int SSR_PIN = 9;

// If your SSR input is active-low, set true:
const bool SSR_ACTIVE_LOW = false;

// -------------------- NTC parameters --------------------
const float SERIES_RESISTOR = 100000.0f;    // fixed resistor
const float NOMINAL_RESISTANCE = 100000.0f; // NTC @25C
const float NOMINAL_TEMPERATURE = 25.0f;    // C
const float BETA_COEFFICIENT = 3950.0f;

// -------------------- Filters --------------------
const float TEMP_FILTER_ALPHA = 0.15f;
const float RATE_FILTER_ALPHA = 0.20f;

// -------------------- PID (0..1 duty) --------------------
const float KP = 8.0f;
const float KI = 0.06f;
const float KD = 2.0f;

const float PID_INTEGRAL_MIN = -50.0f;
const float PID_INTEGRAL_MAX = 50.0f;

// -------------------- SSR windowing --------------------
const unsigned long CONTROL_WINDOW_MS = 1000;

// -------------------- Telemetry --------------------
const unsigned long TELEMETRY_INTERVAL_MS = 500;

// -------------------- Adaptive profile follower (“time-warp”) --------------------
// IMPORTANT FIX: Never allow profile time to run faster than real time.
// Otherwise you enter COOL while you’re still behind and never reached peak.
const float TIME_WARP_GAIN = 0.060f; // was 0.035 (slow down more when behind)
const float MIN_TIME_RATE = 0.10f;   // allow heavy slowing if needed
const float MAX_TIME_RATE = 1.00f;   // CRITICAL: never speed up profile time

// Ramp-limit setpoint (reduces command jumps)
const float MAX_RAMP_UP_C_PER_S = 1.0f;
const float MAX_RAMP_DOWN_C_PER_S = 2.0f;

// -------------------- Warmup gating --------------------
const float PREHEAT_START_RATE_C_PER_S = 0.30f;

// -------------------- Profile --------------------
struct ProfileStep
{
  unsigned long durationMs;
  float startTempC;
  float endTempC;
  const char *label;
};

ProfileStep profile[] = {
    {150000, 25.0f, 150.0f, "PREHEAT"},
    {120000, 150.0f, 180.0f, "SOAK"},
    {45000, 180.0f, 225.0f, "REFLOW"},
    {120000, 225.0f, 50.0f, "COOL"}};

const int PROFILE_COUNT = sizeof(profile) / sizeof(profile[0]);

static unsigned long totalProfileMs()
{
  unsigned long total = 0;
  for (int i = 0; i < PROFILE_COUNT; i++)
    total += profile[i].durationMs;
  return total;
}

bool reflowPeakReached = false;
unsigned long reflowPeakHoldStartMs = 0;

const float REFLOW_PEAK_TEMP_C = 220.0f;       // must actually reach near peak
const float REFLOW_FALL_RATE_CPS = -0.05f;     // cooling has started
const unsigned long REFLOW_MIN_HOLD_MS = 5000; // optional safety (5s)

// -------------------- T1: hard heater cutoff before SOAK --------------------
// Goal: stop heating near end of PREHEAT so you don't carry momentum into SOAK.
const float T1_PREHEAT_CUTOFF_TEMP_C = 143.0f;  // start forcing OFF earlier
const float T1_PREHEAT_RESUME_TEMP_C = 140.0f;  // resume threshold (hysteresis)
const float T1_PREHEAT_CUTOFF_LAST_SEC = 80.0f; // force OFF in last N seconds of PREHEAT

// -------------------- T2: mid-SOAK full blast burst, then PID --------------------
// Goal: in the middle of SOAK slope, go full ON to avoid late REFLOW peak.
// This does NOT wait for actual < setpoint.
const float T2_SOAK_TRIGGER_FRACTION = 0.40f;    // earlier (was 0.50)
const float T2_SOAK_TRIGGER_WINDOW = 0.12f;      // slightly wider window
const float T2_BURST_SECONDS = 32.0f;            // stronger boost (was 18)
const float T2_BLOCK_IF_ABOVE_SETPOINT_C = 2.0f; // don’t burst if already above SP
const float T2_BLOCK_IF_TOO_COLD_C = 15.0f;      // don’t burst if wildly behind (safety / sensor weirdness)

// -------------------- Overshoot guard --------------------
const float OVERSHOOT_OFF_C = 3.0f;
const float OVERSHOOT_ON_C = 1.0f;

// -------------------- State --------------------
enum RunState
{
  IDLE,
  WARMUP,
  RUNNING,
  DONE
};
RunState state = IDLE;

bool running = false;
bool completed = false;

unsigned long lastLoopMs = 0;

// “Profile time” PT (warped) in seconds
float profileTime_s = 0.0f;

// SSR window
unsigned long windowStartMs = 0;

// Telemetry
unsigned long lastTelemetryMs = 0;

// PID
float pidIntegral = 0.0f;
float prevTempForD = 0.0f;
unsigned long lastPidMs = 0;

// Temperature
float filteredTempC = 25.0f;
float filteredRateCps = 0.0f;
float lastRateTemp = 25.0f;
unsigned long lastRateMs = 0;

// Setpoint ramping / overshoot state
float rampedSetpoint = 25.0f;
bool overshootHoldOff = false;

// T1 internal latch (preheat cutoff)
bool t1PreheatHoldOff = false;

// T2 burst state (one-shot per run)
bool t2BurstActive = false;
bool t2BurstDone = false;
unsigned long t2BurstEndMs = 0;

// -------------------- Helpers --------------------
static bool getStepInfo(unsigned long ptMs, int &stepIndex, unsigned long &stepStartMs, unsigned long &stepEndMs)
{
  unsigned long acc = 0;
  for (int i = 0; i < PROFILE_COUNT; i++)
  {
    unsigned long end = acc + profile[i].durationMs;
    if (ptMs <= end)
    {
      stepIndex = i;
      stepStartMs = acc;
      stepEndMs = end;
      return true;
    }
    acc = end;
  }
  stepIndex = PROFILE_COUNT - 1;
  stepStartMs = totalProfileMs() - profile[PROFILE_COUNT - 1].durationMs;
  stepEndMs = totalProfileMs();
  return false;
}

static void ssrWrite(bool enabled)
{
  bool level = enabled ? HIGH : LOW;
  if (SSR_ACTIVE_LOW)
    level = !level;
  digitalWrite(SSR_PIN, level);
}

static float convertAdcToTempC(int adc)
{
  if (adc <= 0)
    adc = 1;
  if (adc >= 1023)
    adc = 1022;

  float r = SERIES_RESISTOR * ((float)adc / (1023.0f - adc));

  float steinhart = r / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0f / (NOMINAL_TEMPERATURE + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;

  if (steinhart < -40.0f)
    steinhart = -40.0f;
  if (steinhart > 350.0f)
    steinhart = 350.0f;
  return steinhart;
}

static float readTempC()
{
  return convertAdcToTempC(analogRead(THERMISTOR_PIN));
}

static float computePidDuty(float setpoint, float tempC)
{
  unsigned long now = millis();
  if (lastPidMs == 0)
  {
    lastPidMs = now;
    prevTempForD = tempC;
  }

  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0.0f)
    dt = 0.001f;

  float error = setpoint - tempC;
  float dTemp = (tempC - prevTempForD) / dt;
  float derivative = -dTemp;

  float output = KP * error + KI * pidIntegral + KD * derivative;

  bool satHigh = output >= 1.0f;
  bool satLow = output <= 0.0f;

  if ((!satHigh || error < 0.0f) && (!satLow || error > 0.0f))
    pidIntegral += error * dt;

  pidIntegral = constrain(pidIntegral, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);

  output = KP * error + KI * pidIntegral + KD * derivative;

  prevTempForD = tempC;
  lastPidMs = now;

  if (output > 1.0f)
    output = 1.0f;
  if (output < 0.0f)
    output = 0.0f;
  return output;
}

static float nominalSetpointFromPT(unsigned long ptMs, const char *&phaseLabel)
{
  unsigned long acc = 0;
  for (int i = 0; i < PROFILE_COUNT; i++)
  {
    unsigned long end = acc + profile[i].durationMs;
    if (ptMs <= end)
    {
      float u = (float)(ptMs - acc) / (float)profile[i].durationMs;
      phaseLabel = profile[i].label;
      return profile[i].startTempC + (profile[i].endTempC - profile[i].startTempC) * u;
    }
    acc = end;
  }
  phaseLabel = "DONE";
  return profile[PROFILE_COUNT - 1].endTempC;
}

static void startProfile()
{
  running = true;
  completed = false;
  state = WARMUP;

  profileTime_s = 0.0f;
  lastLoopMs = millis();

  windowStartMs = millis();
  lastTelemetryMs = 0;

  pidIntegral = 0.0f;
  lastPidMs = 0;

  float t = readTempC();
  filteredTempC = t;
  lastRateTemp = t;
  lastRateMs = 0;
  filteredRateCps = 0.0f;

  rampedSetpoint = profile[0].startTempC;
  overshootHoldOff = false;

  t1PreheatHoldOff = false;

  t2BurstActive = false;
  t2BurstDone = false;
  t2BurstEndMs = 0;
}

static void stopProfile()
{
  running = false;
  completed = false;
  state = IDLE;
  ssrWrite(false);
}

void setup()
{
  pinMode(SSR_PIN, OUTPUT);
  ssrWrite(false);
  Serial.begin(115200);
}

void loop()
{
  // Commands
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("START"))
      startProfile();
    if (cmd.equalsIgnoreCase("ABORT"))
      stopProfile();
  }

  unsigned long now = millis();

  // Read and filter temp
  float t = readTempC();
  filteredTempC = filteredTempC + TEMP_FILTER_ALPHA * (t - filteredTempC);

  // Filtered rate
  if (lastRateMs == 0)
  {
    lastRateMs = now;
    lastRateTemp = filteredTempC;
  }
  else
  {
    float dt = (now - lastRateMs) / 1000.0f;
    if (dt > 0.0f)
    {
      float rate = (filteredTempC - lastRateTemp) / dt;
      filteredRateCps = filteredRateCps + RATE_FILTER_ALPHA * (rate - filteredRateCps);
      lastRateTemp = filteredTempC;
      lastRateMs = now;
    }
  }

  float setpoint = 0.0f;
  const char *phaseLabel = "IDLE";
  bool heaterOn = false;

  if (running)
  {
    float dt_s = (now - lastLoopMs) / 1000.0f;
    if (dt_s < 0.0f)
      dt_s = 0.0f;
    if (dt_s > 1.5f)
      dt_s = 1.5f;
    lastLoopMs = now;

    if (state == WARMUP)
    {
      phaseLabel = "PREHEAT";
      heaterOn = true;
      setpoint = profile[0].startTempC;

      if (filteredRateCps >= PREHEAT_START_RATE_C_PER_S)
      {
        state = RUNNING;
        profileTime_s = 0.0f;
        windowStartMs = now;
        pidIntegral = 0.0f;
        lastPidMs = 0;
        rampedSetpoint = profile[0].startTempC;
        overshootHoldOff = false;
        t1PreheatHoldOff = false;
        t2BurstActive = false;
        t2BurstDone = false;
        t2BurstEndMs = 0;
      }
    }
    else if (state == RUNNING)
    {
      unsigned long ptMs = (unsigned long)(profileTime_s * 1000.0f);
      float nominal = nominalSetpointFromPT(ptMs, phaseLabel);

      bool profileTimeElapsed = (ptMs >= totalProfileMs());

      // Detect reflow peak reached
      if (!reflowPeakReached && filteredTempC >= REFLOW_PEAK_TEMP_C)
      {
        reflowPeakReached = true;
        reflowPeakHoldStartMs = now;
      }

      // Detect cooling
      bool coolingStarted = (filteredRateCps <= REFLOW_FALL_RATE_CPS);

      // Allow exit only if reflow actually happened
      bool reflowComplete =
          reflowPeakReached &&
          (coolingStarted ||
           (now - reflowPeakHoldStartMs >= REFLOW_MIN_HOLD_MS));

      // HARD SAFETY: never run forever
      bool hardTimeout =
          (profileTime_s > (totalProfileMs() / 1000.0f + 60.0f)); // +60s max

      if (profileTimeElapsed && (reflowComplete || hardTimeout))
      {
        running = false;
        completed = true;
        state = DONE;
        ssrWrite(false);
      }

      else
      {
        // Time-warp: slow down if behind, never speed up beyond 1.0x
        float errorNominal = nominal - filteredTempC; // positive if behind
        float timeRate = 1.0f - TIME_WARP_GAIN * errorNominal;
        if (timeRate < MIN_TIME_RATE)
          timeRate = MIN_TIME_RATE;
        if (timeRate > MAX_TIME_RATE)
          timeRate = MAX_TIME_RATE;

        profileTime_s += dt_s * timeRate;

        ptMs = (unsigned long)(profileTime_s * 1000.0f);
        nominal = nominalSetpointFromPT(ptMs, phaseLabel);

        // ramp-limit setpoint
        float maxUp = MAX_RAMP_UP_C_PER_S * dt_s;
        float maxDown = MAX_RAMP_DOWN_C_PER_S * dt_s;

        if (nominal > rampedSetpoint)
          rampedSetpoint = min(nominal, rampedSetpoint + maxUp);
        else
          rampedSetpoint = max(nominal, rampedSetpoint - maxDown);

        setpoint = rampedSetpoint;

        // COOL: heater off
        if (strcmp(phaseLabel, "COOL") == 0)
        {
          heaterOn = false;
          pidIntegral = 0.0f;
          t2BurstActive = false;
        }
        else
        {
          // Step timing
          int stepIndex;
          unsigned long stepStart, stepEnd;
          getStepInfo(ptMs, stepIndex, stepStart, stepEnd);

          float inStep_s = (ptMs - stepStart) / 1000.0f;
          float stepRemaining_s = (stepEnd - ptMs) / 1000.0f;
          float stepDuration_s = max(1.0f, (float)profile[stepIndex].durationMs / 1000.0f);
          float frac = constrain(inStep_s / stepDuration_s, 0.0f, 1.0f);

          // Overshoot guard
          float overshoot = filteredTempC - setpoint;
          if (!overshootHoldOff && overshoot >= OVERSHOOT_OFF_C)
            overshootHoldOff = true;
          if (overshootHoldOff && overshoot <= OVERSHOOT_ON_C)
            overshootHoldOff = false;

          // -------------------- T1: hard cutoff late PREHEAT --------------------
          bool inPreheat = (strcmp(phaseLabel, "PREHEAT") == 0);

          if (inPreheat)
          {
            if (filteredTempC >= T1_PREHEAT_CUTOFF_TEMP_C || stepRemaining_s <= T1_PREHEAT_CUTOFF_LAST_SEC)
              t1PreheatHoldOff = true;

            if (t1PreheatHoldOff && filteredTempC <= T1_PREHEAT_RESUME_TEMP_C)
              t1PreheatHoldOff = false;
          }
          else
          {
            t1PreheatHoldOff = false;
          }

          // -------------------- T2: mid SOAK burst (one-shot), independent of being behind --------------------
          bool inSoak = (strcmp(phaseLabel, "SOAK") == 0);

          bool inT2Window =
              inSoak &&
              !t2BurstDone &&
              (frac >= (T2_SOAK_TRIGGER_FRACTION - T2_SOAK_TRIGGER_WINDOW)) &&
              (frac <= (T2_SOAK_TRIGGER_FRACTION + T2_SOAK_TRIGGER_WINDOW));

          float err = setpoint - filteredTempC; // positive if behind

          bool blockBecauseTooHot = (filteredTempC > (setpoint + T2_BLOCK_IF_ABOVE_SETPOINT_C));
          bool blockBecauseTooCold = (err > T2_BLOCK_IF_TOO_COLD_C);

          if (inT2Window && !blockBecauseTooHot && !blockBecauseTooCold &&
              !overshootHoldOff && !t1PreheatHoldOff && !t2BurstActive)
          {
            t2BurstActive = true;
            t2BurstDone = true;
            t2BurstEndMs = now + (unsigned long)(T2_BURST_SECONDS * 1000.0f);
          }

          if (t2BurstActive && (long)(now - t2BurstEndMs) >= 0)
          {
            t2BurstActive = false;
          }

          // -------------------- Control decisions --------------------
          if (overshootHoldOff)
          {
            heaterOn = false;
            pidIntegral = 0.0f;
            t2BurstActive = false;
          }
          else if (t1PreheatHoldOff)
          {
            heaterOn = false;
            pidIntegral = 0.0f;
            t2BurstActive = false;
          }
          else if (t2BurstActive)
          {
            heaterOn = true; // full blast
          }
          else
          {
            float duty = computePidDuty(setpoint, filteredTempC);

            if (now - windowStartMs >= CONTROL_WINDOW_MS)
              windowStartMs = now;

            unsigned long onTime = (unsigned long)(duty * CONTROL_WINDOW_MS);
            heaterOn = ((now - windowStartMs) < onTime);
          }
        }
      }
    }
    else
    {
      heaterOn = false;
      setpoint = filteredTempC;
      phaseLabel = "IDLE";
      t2BurstActive = false;
    }
  }

  // Apply heater output
  if (running && state == RUNNING)
    ssrWrite(heaterOn);
  else if (running && state == WARMUP)
    ssrWrite(true);
  else
    ssrWrite(false);

  // Telemetry
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryMs = now;

    Serial.print("T:");
    Serial.print(filteredTempC, 2);

    Serial.print(";S:");
    Serial.print(setpoint, 2);

    Serial.print(";STATE:");
    if (running)
      Serial.print("RUNNING");
    else
      Serial.print(completed ? "DONE" : "IDLE");

    Serial.print(";PHASE:");
    Serial.print(phaseLabel);

    Serial.print(";PT:");
    Serial.print(profileTime_s, 1);

    Serial.print(";R:");
    Serial.print(filteredRateCps, 2);

    Serial.print(";T2:");
    Serial.print(t2BurstActive ? "1" : "0");

    Serial.println();
  }
}
